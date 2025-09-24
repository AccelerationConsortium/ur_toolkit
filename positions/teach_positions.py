#!/usr/bin/env python3
"""
Position Teaching CLI
Teach robot positions associated with AprilTag markers for grasping/releasing operations
"""

import argparse
import time
import yaml
from datetime import datetime
from pathlib import Path
import cv2
import numpy as np

import sys

# Add parent directory and setup directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))
sys.path.insert(0, str(Path(__file__).parent.parent / "setup"))

from config_manager import config, get_robot_ip, get_camera_host, get_camera_port

# Import robot and detection modules
from robots.ur.ur_controller import URController
from apriltag_detection import AprilTagDetector

# Add camera module to Python path
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), 'camera'))
from picam import PiCam, PiCamConfig


class PositionTeacher:
    """Teach and manage robot positions associated with AprilTag markers"""

    def __init__(self, robot_ip=None, camera_host=None, camera_port=None):
        """Initialize position teacher with robot and camera connections"""
        self.robot_ip = robot_ip or get_robot_ip()
        self.camera_host = camera_host or get_camera_host()
        self.camera_port = camera_port or get_camera_port()

        # Initialize components
        self.robot = None
        self.camera = None
        self.detector = None
        self.positions_file = config.resolve_path('taught_positions.yaml')

        print("🎯 Position Teaching System")
        print(f"📍 Robot: {self.robot_ip}")
        print(f"📹 Camera: {self.camera_host}:{self.camera_port}")
        print(f"💾 Positions file: {self.positions_file}")

        # Load existing positions
        self.taught_positions = self.load_positions()

    def connect(self):
        """Connect to robot and camera systems"""
        print("\n🔌 Connecting to systems...")

        # Connect to robot in read-only mode (only reads poses, no movement commands)
        try:
            self.robot = URController(self.robot_ip, read_only=True)
            if not self.robot.test_connection():
                raise ConnectionError("Failed to connect to robot")
            print("✅ Robot connected (read-only mode - safe for manual teaching)")
        except Exception as e:
            print(f"❌ Robot connection failed: {e}")
            print("💡 Tip: Enable 'Remote Control' on teach pendant, then use local freedrive for teaching")
            return False

        # Connect to camera (optional for testing)
        try:
            camera_config = PiCamConfig(hostname=self.camera_host, port=self.camera_port)
            self.camera = PiCam(camera_config)
            if not self.camera.test_connection():
                raise ConnectionError("Failed to connect to camera")
            print("✅ Camera connected")
        except Exception as e:
            print(f"⚠️  Camera connection failed: {e}")
            print("🔄 Continuing without camera for robot-only testing...")
            self.camera = None

        # Initialize AprilTag detector (optional without camera)
        try:
            self.detector = AprilTagDetector()
            print("✅ AprilTag detector initialized")
        except Exception as e:
            print(f"⚠️  AprilTag detector failed: {e}")
            print("🔄 Continuing without AprilTag detection...")
            self.detector = None

        return True

    def connect_for_freedrive(self):
        """Connect to robot with full control for freedrive teaching"""
        print("\n🔌 Connecting with freedrive control...")

        # Close existing connection if exists
        if self.robot:
            self.robot.close()

        # Connect to robot with full control for freedrive
        try:
            self.robot = URController(self.robot_ip, read_only=False)
            if not self.robot.test_connection():
                raise ConnectionError("Failed to connect to robot")
            print("✅ Robot connected with freedrive capability")
            return True
        except Exception as e:
            print(f"❌ Freedrive connection failed: {e}")
            print("💡 Ensure robot is in Remote Control mode on teach pendant")
            return False

    def enable_freedrive(self):
        """Enable freedrive mode for manual positioning"""
        if not self.robot or not hasattr(self.robot, 'rtde_c') or not self.robot.rtde_c:
            print("❌ Robot not connected with control interface")
            return False

        try:
            print("🔓 Enabling freedrive mode...")
            self.robot.rtde_c.teachMode()
            print("✅ Freedrive enabled - you can now move the robot manually")
            return True
        except Exception as e:
            print(f"❌ Failed to enable freedrive: {e}")
            return False

    def disable_freedrive(self):
        """Disable freedrive mode and lock robot position"""
        if not self.robot or not hasattr(self.robot, 'rtde_c') or not self.robot.rtde_c:
            return False

        try:
            print("🔒 Disabling freedrive mode...")
            self.robot.rtde_c.endTeachMode()
            print("✅ Freedrive disabled - robot position locked")
            return True
        except Exception as e:
            print(f"❌ Failed to disable freedrive: {e}")
            return False

    def connect_for_movement(self):
        """Connect to robot with full control for movement commands"""
        print("\n🔌 Connecting with movement control...")

        # Close existing read-only connection if exists
        if self.robot:
            self.robot.close()

        # Connect to robot with full control
        try:
            self.robot = URController(self.robot_ip, read_only=False)
            if not self.robot.test_connection():
                raise ConnectionError("Failed to connect to robot")
            print("✅ Robot connected with movement control")
            return True
        except Exception as e:
            print(f"❌ Robot movement connection failed: {e}")
            print("💡 Ensure robot is in Remote Control mode on teach pendant")
            return False

    def disconnect(self):
        """Disconnect from systems"""
        if self.robot:
            self.robot.close()
        print("🔌 Disconnected from systems")

    def load_positions(self):
        """Load taught positions from file"""
        if not self.positions_file.exists():
            return {'apriltags': {}, 'positions': {}}

        try:
            with open(self.positions_file, 'r') as f:
                data = yaml.safe_load(f) or {}

            # Ensure we have both sections
            if 'apriltags' not in data:
                data['apriltags'] = {}
            if 'positions' not in data:
                data['positions'] = {}

            positions_count = len(data['positions'])
            apriltags_count = len(data['apriltags'])
            print(f"📖 Loaded {positions_count} taught positions and {apriltags_count} AprilTags")
            return data
        except Exception as e:
            print(f"⚠️  Error loading positions: {e}")
            return {'apriltags': {}, 'positions': {}}

    def save_positions(self):
        """Save taught positions to file in readable format with inline arrays"""
        try:
            # Ensure directory exists
            self.positions_file.parent.mkdir(parents=True, exist_ok=True)

            # Create data structure with rounded coordinates for readability
            positions_data = {}
            for name, pos_data in self.taught_positions.get('positions', {}).items():
                cleaned_data = pos_data.copy()

                # Round coordinates and joints to 3 decimal places for readability
                if 'coordinates' in cleaned_data:
                    cleaned_data['coordinates'] = [round(x, 3) for x in cleaned_data['coordinates']]
                if 'joints' in cleaned_data:
                    cleaned_data['joints'] = [round(x, 3) for x in cleaned_data['joints']]
                if 'camera_to_tag' in cleaned_data and cleaned_data['camera_to_tag']:
                    cleaned_data['camera_to_tag'] = [round(x, 3) for x in cleaned_data['camera_to_tag']]

                # Remove usage tracking fields for cleaner output
                cleaned_data.pop('usage_count', None)
                cleaned_data.pop('last_used', None)

                positions_data[name] = cleaned_data

            data = {
                'apriltags': self.taught_positions.get('apriltags', {}),
                'positions': positions_data
            }

            # Custom YAML representation for better readability
            def represent_list(dumper, data):
                # Use inline style for coordinate arrays (6 elements) and joint arrays
                if len(data) == 6 and all(isinstance(x, (int, float)) for x in data):
                    return dumper.represent_sequence('tag:yaml.org,2002:seq', data, flow_style=True)
                return dumper.represent_sequence('tag:yaml.org,2002:seq', data, flow_style=False)

            yaml.add_representer(list, represent_list)

            # Save with custom formatting
            yaml_content = yaml.dump(data, default_flow_style=False, sort_keys=False,
                                     indent=2, width=120, allow_unicode=True)

            # Add extra spacing between position entries for better readability
            lines = yaml_content.split('\n')
            formatted_lines = []
            in_positions = False
            last_was_position = False

            for line in lines:
                if line.strip() == 'positions:':
                    in_positions = True
                    formatted_lines.append(line)
                elif in_positions and line and not line.startswith(' '):
                    # End of positions section
                    in_positions = False
                    formatted_lines.append(line)
                    last_was_position = False
                elif in_positions and line and line.startswith('  ') and not line.startswith('    '):
                    # This is a position name line
                    if last_was_position:
                        formatted_lines.append('')  # Add empty line before new position
                    formatted_lines.append(line)
                    last_was_position = True
                else:
                    formatted_lines.append(line)
                    if in_positions and line.strip():
                        last_was_position = False

            with open(self.positions_file, 'w') as f:
                f.write('\n'.join(formatted_lines))

            print(f"💾 Saved positions to {self.positions_file}")

        except Exception as e:
            print(f"❌ Error saving positions: {e}")

    def capture_current_state(self):
        """Capture current robot pose and camera view with AprilTag detection"""
        # Get current robot pose
        robot_pose = self.robot.get_tcp_pose()
        joint_positions = self.robot.get_joint_positions()

        # Capture camera image and detect AprilTags
        photo_path = self.camera.capture_photo()
        image = cv2.imread(photo_path)

        if image is None:
            raise ValueError(f"Failed to load captured image: {photo_path}")

        # Detect AprilTags
        detections = self.detector.detect_tags(image)

        return {
            'robot_pose': robot_pose.tolist(),
            'joint_positions': joint_positions.tolist(),
            'image_path': str(photo_path),
            'apriltag_detections': detections,
            'timestamp': datetime.now().isoformat()
        }

    def _get_equipment_observation_poses(self, equipment_name):
        """Get all observation poses for a specific equipment"""
        observation_poses = []
        if hasattr(self.taught_positions, 'positions'):
            for pos_name, pos_data in self.taught_positions['positions'].items():
                if pos_data.get('pose_type') == 'observation' and pos_data.get('equipment_name') == equipment_name:
                    observation_poses.append(pos_name)
        return observation_poses

    def _offer_observation_pose_teaching(self, position_name, equipment_name):
        """Offer to teach an observation pose if none exists for the equipment"""
        if not equipment_name:
            return  # No equipment specified, can't offer observation pose

        # Check if observation poses already exist for this equipment
        existing_obs_poses = self._get_equipment_observation_poses(equipment_name)

        if existing_obs_poses:
            print(f"ℹ️  Existing observation poses for '{equipment_name}': {', '.join(existing_obs_poses)}")
            return  # Observation poses already exist

        print(f"\n💡 No observation poses found for equipment '{equipment_name}'")
        print("📸 Observation poses enable visual servoing for positions without direct AprilTag view")

        try:
            teach_obs = input("🔍 Would you like to teach an observation pose for this equipment? (Y/n): ").lower().strip()
            if teach_obs != 'n' and teach_obs != 'no':
                # Generate suggested observation pose name
                obs_name = f"{equipment_name}-observe"
                suggested_name = input(f"Observation pose name (default: {obs_name}): ").strip()
                if not suggested_name:
                    suggested_name = obs_name

                obs_description = input("Observation pose description (optional): ").strip()
                if not obs_description:
                    obs_description = f"Observation pose for {equipment_name}"

                print(f"\n🎯 Teaching observation pose '{suggested_name}' for equipment '{equipment_name}'...")
                success = self.teach_observation_pose(suggested_name, equipment_name, obs_description)

                if success:
                    print(f"✅ Observation pose '{suggested_name}' taught successfully!")
                    print(f"💡 Future positions for '{equipment_name}' can now use this observation pose for visual servoing")
                else:
                    print(f"❌ Failed to teach observation pose '{suggested_name}'")
        except (EOFError, KeyboardInterrupt):
            print("\n💡 Skipping observation pose teaching")

    def _handle_equipment_association(self, position_name, state):
        """Handle equipment association and observation pose teaching after position is taught"""
        print(f"\n⚙️  Equipment Association for '{position_name}'")
        print("=" * 50)

        try:
            # Ask for equipment name
            equipment_name = input("📦 Equipment name (required for visual servoing): ").strip()
            if not equipment_name:
                print("⚠️  No equipment specified - position will not have visual servoing capability")
                return None, None

            # Check if we have existing observation poses for this equipment
            existing_obs_poses = self._get_equipment_observation_poses(equipment_name)

            observation_pose = None

            if existing_obs_poses:
                print(f"✅ Found existing observation poses for '{equipment_name}': {', '.join(existing_obs_poses)}")

                # Let user choose which observation pose to use
                print("\nOptions:")
                for i, obs_pose in enumerate(existing_obs_poses, 1):
                    print(f"  {i}. Use '{obs_pose}'")
                print(f"  {len(existing_obs_poses) + 1}. Teach NEW observation pose")
                print(f"  {len(existing_obs_poses) + 2}. No observation pose (manual positioning only)")

                while True:
                    choice = input(f"Select option (1-{len(existing_obs_poses) + 2}): ").strip()
                    if choice.isdigit():
                        idx = int(choice) - 1
                        if 0 <= idx < len(existing_obs_poses):
                            # Use existing observation pose
                            observation_pose = existing_obs_poses[idx]
                            break
                        elif idx == len(existing_obs_poses):
                            # Teach new observation pose
                            observation_pose = self._teach_new_observation_pose(equipment_name)
                            break
                        elif idx == len(existing_obs_poses) + 1:
                            # No observation pose
                            observation_pose = None
                            break
                    print(f"❌ Please enter a number between 1 and {len(existing_obs_poses) + 2}")
            else:
                # No existing observation pose for this equipment
                print(f"\n🎯 No observation pose found for equipment '{equipment_name}'")

                # Check if current position has AprilTag detection
                has_apriltag = bool(state.get('apriltag_detections'))

                if has_apriltag:
                    print("✅ AprilTag detected in current position!")
                    print("💡 This position can serve as both work position AND observation pose")

                    # Ask negative question - default to using dual purpose
                    teach_separate = input("🔍 Teach a SEPARATE observation pose instead? (y/N): ").lower().strip()
                    if teach_separate == 'y' or teach_separate == 'yes':
                        observation_pose = self._teach_new_observation_pose(equipment_name)
                    else:
                        print(f"🎯 Using '{position_name}' as dual-purpose position (work + observation)")
                        observation_pose = position_name  # Use current position as observation pose
                else:
                    print("📸 An observation pose enables visual servoing for this position")

                    # Ask positive question when no AprilTag detected
                    teach_obs = input("🔍 Would you like to teach an observation pose for this equipment? (Y/n): ").lower().strip()
                    if teach_obs != 'n' and teach_obs != 'no':
                        observation_pose = self._teach_new_observation_pose(equipment_name)

            # Update the position with equipment association
            self._update_position_with_equipment(position_name, equipment_name, observation_pose, state)

            return equipment_name, observation_pose

        except (EOFError, KeyboardInterrupt):
            print("\n💡 Skipping equipment association")
            return None, None

    def _teach_new_observation_pose(self, equipment_name):
        """Helper to teach a new observation pose for equipment"""
        # Generate suggested observation pose name
        obs_name = f"{equipment_name}-observe"
        suggested_name = input(f"Observation pose name (default: {obs_name}): ").strip()
        if not suggested_name:
            suggested_name = obs_name

        obs_description = input("Observation pose description (optional): ").strip()
        if not obs_description:
            obs_description = f"Observation pose for {equipment_name}"

        print(f"\n🎯 Teaching observation pose '{suggested_name}' for equipment '{equipment_name}'...")
        success = self.teach_observation_pose(suggested_name, equipment_name, obs_description)

        if success:
            print(f"✅ Observation pose '{suggested_name}' taught successfully!")
            return suggested_name
        else:
            print(f"❌ Failed to teach observation pose '{suggested_name}'")
            return None

    def _update_position_with_equipment(self, position_name, equipment_name, observation_pose, state):
        """Update position with equipment association and observation pose data"""
        if position_name not in self.taught_positions.get('positions', {}):
            print(f"❌ Position '{position_name}' not found")
            return

        position_data = self.taught_positions['positions'][position_name]

        # Add equipment name
        if equipment_name:
            position_data['equipment_name'] = equipment_name

        # Add observation pose and calculate offset if applicable
        if observation_pose:
            position_data['observation_pose'] = observation_pose

            # Calculate offset from observation pose
            obs_pos_data = self.taught_positions['positions'].get(observation_pose)
            if obs_pos_data and 'coordinates' in obs_pos_data:
                try:
                    import numpy as np
                    obs_coords = np.array(obs_pos_data['coordinates'])
                    current_coords = np.array(position_data['coordinates'])
                    observation_offset = (current_coords - obs_coords).tolist()
                    position_data['observation_offset'] = observation_offset
                    print(f"🧮 Calculated offset from observation pose: {[round(x, 3) for x in observation_offset]}")
                except Exception as e:
                    print(f"⚠️  Could not calculate offset: {e}")
                    position_data['observation_offset'] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            else:
                position_data['observation_offset'] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        else:
            position_data['observation_pose'] = None
            position_data['observation_offset'] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Handle AprilTag detection and dual-purpose positions
        has_direct_apriltag_view = bool(state.get('apriltag_detections'))

        # Check if this is explicitly a dual-purpose position (observation_pose == position_name)
        is_dual_purpose = observation_pose == position_name

        if is_dual_purpose and has_direct_apriltag_view:
            # Explicitly set as dual-purpose position
            print(f"🎯 Setting '{position_name}' as dual-purpose position (work + observation)")

            if len(state['apriltag_detections']) == 1:
                selected_apriltag = state['apriltag_detections'][0]
                tag_id = selected_apriltag['tag_id']
                position_data['tag_reference'] = f"tag_{tag_id}"
                position_data['camera_to_tag'] = self._extract_camera_to_tag_transform(selected_apriltag)
                position_data['pose_type'] = ['work', 'observation']
                # Self-reference for observation
                position_data['observation_pose'] = position_name
                position_data['observation_offset'] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                print(f"✅ Dual-purpose position with Tag ID {tag_id}")
            else:
                # Multiple tags - fallback to position only
                position_data['pose_type'] = 'work'
                print("🏷️  Multiple AprilTags detected - defaulting to work-only position")

        elif has_direct_apriltag_view and equipment_name:
            # Check if this can be auto-detected as dual purpose (no other observation poses exist)
            existing_obs_poses = self._get_equipment_observation_poses(equipment_name)
            # Remove self from existing poses if present
            existing_obs_poses = [obs for obs in existing_obs_poses if obs != position_name]

            if not existing_obs_poses and not observation_pose:
                # No other observation pose exists and none specified - this can be dual purpose
                print(f"🎯 Auto-setting '{position_name}' as dual-purpose (no other observation poses exist)")

                if len(state['apriltag_detections']) == 1:
                    selected_apriltag = state['apriltag_detections'][0]
                    tag_id = selected_apriltag['tag_id']
                    position_data['tag_reference'] = f"tag_{tag_id}"
                    position_data['camera_to_tag'] = self._extract_camera_to_tag_transform(selected_apriltag)
                    position_data['pose_type'] = ['work', 'observation']
                    # Self-reference for observation
                    position_data['observation_pose'] = position_name
                    position_data['observation_offset'] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                    print(f"✅ Auto dual-purpose position with Tag ID {tag_id}")
                else:
                    # Multiple tags - position only
                    position_data['pose_type'] = 'work'
                    print("🏷️  Multiple AprilTags - work-only position")
            else:
                # Other observation poses exist or one was specified - this is work position only
                position_data['pose_type'] = 'work'
                if existing_obs_poses:
                    print(f"📍 Work-only position (existing observation poses: {', '.join(existing_obs_poses)})")
        else:
            # No AprilTag view or no equipment - work position only
            position_data['pose_type'] = 'work'

        # Clean up irrelevant fields for work positions
        if position_data.get('pose_type') == 'work' or (isinstance(position_data.get('pose_type'), list) and 'work' in position_data['pose_type'] and 'observation' not in position_data['pose_type']):
            # Remove AprilTag data from work-only positions
            position_data.pop('tag_reference', None)
            position_data.pop('has_apriltag_view', None)
            position_data.pop('camera_to_tag', None)

        # Remove irrelevant metadata fields
        position_data.pop('taught_with_freedrive', None)
        position_data.pop('timestamp', None)

        # Save the updated position
        self.save_positions()
        print(f"✅ Position '{position_name}' updated with equipment association")

        if observation_pose:
            print(f"🔗 Linked to observation pose: '{observation_pose}'")
        if equipment_name:
            print(f"⚙️  Equipment: '{equipment_name}'")

    def teach_position(self, position_name, description="", interactive_metadata=True):
        """Teach a new position using manual freedrive on teach pendant"""
        print(f"\n🎯 Teaching position: {position_name}")

        # Check if position already exists
        existing_position = None
        if hasattr(self.taught_positions, 'positions') and position_name in self.taught_positions.get('positions', {}):
            existing_position = self.taught_positions['positions'][position_name]

            print(f"\n⚠️  Position '{position_name}' already exists!")
            print("📋 Current position details:")
            print(f"   📝 Description: {existing_position.get('description', 'No description')}")
            print(f"   🎯 Type: {existing_position.get('position_type', 'Unknown')}")
            print(f"   ⭐ Priority: {existing_position.get('priority', 'Unknown')}")
            print(f"   📅 Taught: {existing_position.get('timestamp', 'Unknown')}")
            if existing_position.get('equipment'):
                print(f"   ⚙️  Equipment: {existing_position.get('equipment')}")
            if existing_position.get('safety_notes'):
                print(f"   ⚠️  Safety: {existing_position.get('safety_notes')}")

            print("\n🔄 Options:")
            print("  Y: Overwrite existing position (old data will be lost)")
            print("  N: Cancel teaching (keep existing position)")

            choice = input("Overwrite existing position? (Y/N): ").strip().upper()
            if choice != 'Y':
                print(f"❌ Teaching cancelled. Position '{position_name}' unchanged.")
                return
            else:
                print(f"✅ Proceeding to overwrite position '{position_name}'...")

        # Collect metadata interactively if not provided
        metadata = {
            'description': description,
            'equipment_name': None,
            'position_type': 'grasp',  # Default type
            'priority': 'normal',
            'safety_notes': ''
        }

        if interactive_metadata and not description:
            print("\n📝 Position Metadata Collection")
            print("=" * 40)

            # Description
            metadata['description'] = input("📝 Description (optional): ").strip()

            # Equipment association
            equipment = input("⚙️  Associated equipment (optional): ").strip()
            if equipment:
                metadata['equipment_name'] = equipment

            # Position type
            print("\n🎯 Position type:")
            print("  1. Grasp (pick up objects)")
            print("  2. Place (put down objects)")
            print("  3. Inspect (check/verify)")
            print("  4. Process (operate on objects)")
            print("  5. Home (safe/parking position)")
            print("  6. Other")

            type_choice = input("Select type (1-6, default: 1): ").strip()
            type_map = {
                '1': 'grasp', '2': 'place', '3': 'inspect',
                '4': 'process', '5': 'home', '6': 'other'
            }
            metadata['position_type'] = type_map.get(type_choice, 'grasp')

            # Priority level
            print("\n⭐ Priority level:")
            print("  1. Critical (essential for operation)")
            print("  2. High (important for efficiency)")
            print("  3. Normal (standard operation)")
            print("  4. Low (convenience/optional)")

            priority_choice = input("Select priority (1-4, default: 3): ").strip()
            priority_map = {
                '1': 'critical', '2': 'high', '3': 'normal', '4': 'low'
            }
            metadata['priority'] = priority_map.get(priority_choice, 'normal')

            # Safety notes
            safety = input("⚠️  Safety notes (optional): ").strip()
            if safety:
                metadata['safety_notes'] = safety

            print()

        print(f"📝 Description: {metadata['description'] or 'No description'}")
        if metadata['equipment_name']:
            print(f"⚙️  Equipment: {metadata['equipment_name']}")
        print(f"🎯 Type: {metadata['position_type'].title()}")
        print(f"⭐ Priority: {metadata['priority'].title()}")
        if metadata['safety_notes']:
            print(f"⚠️  Safety: {metadata['safety_notes']}")
        print("🤲 Using manual freedrive on teach pendant...")

        try:
            print("\n" + "=" * 60)
            print("🤲 MANUAL FREEDRIVE MODE")
            print("=" * 60)
            print("📍 Use the teach pendant to manually move the robot arm")
            print("🎯 Position the robot where you want to teach the position")
            print("📷 The camera will capture the scene when you confirm")
            print("⚠️  Make sure the robot is in local control mode!")
            print("=" * 60)

            # Wait for user to position robot manually
            input("Press ENTER when the robot is in the correct position...")

            # Get current robot state from RTDE
            current_pose = self.robot.get_tcp_pose()
            current_joints = self.robot.get_joint_positions()

            print("\n📊 Current robot position:")
            print(f"   TCP Pose: {self.robot.format_pose(current_pose)}")
            print(f"   Joints: {current_joints.round(3)} rad")

            # Confirm position before capturing scene
            confirm = input("\n💾 Capture scene and save this position? (y/N): ").lower().strip()
            if confirm != 'y':
                print("❌ Position teaching cancelled")
                return False

            print("\n📷 Capturing scene...")

            # Capture current state (camera image and AprilTag detection)
            state = self.capture_current_state()

            # Update state with manually positioned robot data
            state['robot_pose'] = current_pose
            state['joint_positions'] = current_joints

            # Create initial position data
            position_data = {
                'coordinates': current_pose.tolist(),
                'joints': current_joints.round(3).tolist(),
                'description': metadata['description'],
                'pose_type': 'work'
            }

            # Save initial position first
            self.taught_positions['positions'][position_name] = position_data
            self.save_positions()
            print(f"✅ Position '{position_name}' taught successfully")

            # Handle equipment association and observation pose setup
            self._handle_equipment_association(position_name, state)

            # Prompt for safe offset position after successful teaching
            try:
                create_safe = input("\n🛡️  Create associated safe position? (Y/n): ").lower().strip()
                if create_safe != 'n' and create_safe != 'no':
                    self.create_simple_safe_offset(position_name)
            except (EOFError, KeyboardInterrupt):
                print("\n💡 Skipping safe position creation")

            return True

        except KeyboardInterrupt:
            print("\n⚠️  Position teaching interrupted")
            return False
        except Exception as e:
            print(f"❌ Failed to teach position: {e}")
            return False

    def teach_position_with_freedrive(self, position_name, description=""):
        """
        Teach a position using remote freedrive mode with simple blocking input.
        This allows manual positioning while robot stays in remote control.
        """
        try:
            print(f"\n🎯 Teaching position with freedrive: {position_name}")
            if description:
                print(f"📄 Description: {description}")

            # Check if position already exists
            if position_name in self.taught_positions.get('positions', {}):
                overwrite = input(f"⚠️  Position '{position_name}' exists. Overwrite? (y/N): ").lower().strip()
                if overwrite != 'y':
                    print("❌ Position teaching cancelled")
                    return False

            # Connect with freedrive capability
            if not self.connect_for_freedrive():
                return False

            # Get initial state
            initial_pose = self.robot.get_tcp_pose()
            initial_joints = self.robot.get_joint_positions()

            print(f"📍 Initial pose: {self.robot.format_pose(initial_pose)}")
            print(f"🔧 Initial joints: {initial_joints.round(3)} rad")

            # Enable freedrive mode
            if not self.enable_freedrive():
                return False

            try:
                print("\n🤲 FREEDRIVE MODE ENABLED")
                print("=" * 60)
                print("👋 Move the robot manually to the desired position")
                print("📍 When ready, press ENTER to capture position")
                print("❌ Type 'cancel' and ENTER to abort")
                print("=" * 60)

                # Simple blocking input - no timer, no threading
                user_input = input("").strip().lower()

                if user_input == 'cancel':
                    print("❌ Position teaching cancelled")
                    return False

                # Capture position (any other input including empty/ENTER)
                print("📸 Capturing position...")

                # Capture final position
                final_pose = self.robot.get_tcp_pose()
                final_joints = self.robot.get_joint_positions()

                print("📸 Position captured!")
                print(f"📍 Final pose: {self.robot.format_pose(final_pose)}")
                print(f"🔧 Final joints: {final_joints.round(3)} rad")

                final_movement = np.linalg.norm(final_pose[:3] - initial_pose[:3])
                print(f"📏 Total movement: {final_movement * 1000:.1f}mm")

                # Capture current state for equipment association
                print("\n📷 Capturing scene for equipment association...")
                state = self.capture_current_state()
                state['robot_pose'] = final_pose
                state['joint_positions'] = final_joints

                # Create initial position data
                position_data = {
                    'coordinates': final_pose.tolist(),
                    'joints': final_joints.round(3).tolist(),
                    'description': description,
                    'pose_type': 'work'
                }

                # Save initial position
                if 'positions' not in self.taught_positions:
                    self.taught_positions['positions'] = {}
                self.taught_positions['positions'][position_name] = position_data
                self.save_positions()

                print(f"✅ Position '{position_name}' taught successfully with freedrive!")

                # Handle equipment association and observation pose setup
                self._handle_equipment_association(position_name, state)

                # Prompt for safe offset position after successful teaching
                try:
                    create_safe = input("\n🛡️  Create associated safe position? (Y/n): ").lower().strip()
                    if create_safe != 'n' and create_safe != 'no':
                        self.create_simple_safe_offset(position_name)
                except (EOFError, KeyboardInterrupt):
                    print("\n💡 Skipping safe position creation")

                return True

            except KeyboardInterrupt:
                print("\n❌ Teaching interrupted by user")
                return False

        except Exception as e:
            print(f"❌ Failed to teach position with freedrive: {e}")
            return False

        finally:
            # Always disable freedrive on exit
            try:
                self.disable_freedrive()
                print("🔒 Freedrive mode disabled")
            except Exception as e:
                print(f"⚠️  Warning: Could not disable freedrive: {e}")

    def teach_observation_pose(self, position_name, equipment_name, description=""):
        """Teach an observation pose linked to specific equipment"""
        print(f"\n🔍 Teaching observation pose: {position_name}")

        # Check if position already exists
        existing_position = None
        if hasattr(self.taught_positions, 'positions') and position_name in self.taught_positions.get('positions', {}):
            existing_position = self.taught_positions['positions'][position_name]

            print(f"\n⚠️  Observation pose '{position_name}' already exists!")
            print("📋 Current observation pose details:")
            print(f"   📝 Description: {existing_position.get('description', 'No description')}")
            print(f"   ⚙️  Equipment: {existing_position.get('equipment_name', 'Unknown')}")
            print(f"   🏷️  AprilTag: {existing_position.get('tag_reference', 'None')}")
            print(f"   📅 Taught: {existing_position.get('timestamp', 'Unknown')}")

            print("\n🔄 Options:")
            print("  Y: Overwrite existing observation pose (old data will be lost)")
            print("  N: Cancel teaching (keep existing observation pose)")

            choice = input("Overwrite existing observation pose? (Y/N): ").strip().upper()
            if choice != 'Y':
                print(f"❌ Teaching cancelled. Observation pose '{position_name}' unchanged.")
                return False
            else:
                print(f"✅ Proceeding to overwrite observation pose '{position_name}'...")

        print(f"⚙️  Equipment: {equipment_name}")
        print(f"📝 Description: {description}")
        print("🎯 This pose should provide clear AprilTag visibility for the equipment")

        try:
            print("\n" + "=" * 60)
            print("🔍 OBSERVATION POSE MODE")
            print("=" * 60)

            # Connect with freedrive capability
            if not self.connect_for_freedrive():
                print("❌ Failed to establish freedrive connection")
                return False

            # Enable freedrive mode for easier positioning
            print("🔧 Enabling freedrive mode for easier positioning...")
            freedrive_enabled = False
            try:
                # Attempt to enable freedrive mode
                freedrive_enabled = self.enable_freedrive()
                if freedrive_enabled:
                    print("🎮 You can now move the robot manually")
                else:
                    print("⚠️  Could not enable freedrive mode")
                    print("📍 Please enable freedrive mode on the teach pendant manually")
                    print("💡 Make sure 'Remote Control' is enabled on teach pendant")
            except Exception as e:
                print(f"⚠️  Could not enable freedrive mode: {e}")
                print("📍 Please enable freedrive mode on the teach pendant manually")
                print("💡 Make sure 'Remote Control' is enabled on teach pendant")

            print("🎯 Position the robot to observe AprilTags on the equipment")
            print("📷 The camera MUST detect AprilTags from this position")
            print("💡 Tip: Enable 'Remote Control' on teach pendant if not already enabled")
            print("=" * 60)

            # Wait for user to position robot manually
            input("Press ENTER when the robot has clear view of equipment AprilTags...")

            # Get current robot state from RTDE
            current_pose = self.robot.get_tcp_pose()
            current_joints = self.robot.get_joint_positions()

            print("\n📊 Current robot position:")
            print(f"   TCP Pose: {self.robot.format_pose(current_pose)}")
            print(f"   Joints: {current_joints.round(3)} rad")

            # Confirm position before capturing scene
            confirm = input("\n📷 Capture scene and validate AprilTag detection? (y/N): ").lower().strip()
            if confirm != 'y':
                print("❌ Observation pose teaching cancelled")
                return False

            print("\n📷 Capturing scene and detecting AprilTags...")

            # Capture current state (camera image and AprilTag detection)
            state = self.capture_current_state()

            # Update state with manually positioned robot data
            state['robot_pose'] = current_pose
            state['joint_positions'] = current_joints

            # MANDATORY: Must detect AprilTags for observation pose
            if not state['apriltag_detections']:
                print("❌ CRITICAL: No AprilTags detected from this position!")
                print("🔍 Observation poses MUST have clear AprilTag visibility.")
                print("💡 Adjust robot position to get AprilTags in camera view and try again.")
                return False

            # Display detected AprilTags
            print(f"✅ Detected {len(state['apriltag_detections'])} AprilTag(s):")

            # Display all detected tags with indices
            for i, det in enumerate(state['apriltag_detections'], 1):
                tag_id = det['tag_id']
                distance = det.get('distance_mm', 'unknown')
                print(f"   {i}. Tag ID {tag_id}: {distance}mm away")

            # Select which AprilTag to associate with this observation pose
            selected_apriltag = None
            if len(state['apriltag_detections']) == 1:
                # Only one tag - use it automatically
                selected_apriltag = state['apriltag_detections'][0]
                print(f"✅ Using Tag ID {selected_apriltag['tag_id']} (only tag detected)")
            else:
                # Multiple tags - ask user to choose
                while True:
                    choice = input(f"Select PRIMARY AprilTag for this equipment (1-{len(state['apriltag_detections'])}): ").strip()
                    if choice.isdigit():
                        idx = int(choice) - 1
                        if 0 <= idx < len(state['apriltag_detections']):
                            selected_apriltag = state['apriltag_detections'][idx]
                            print(f"✅ Selected Tag ID {selected_apriltag['tag_id']} as primary reference")
                            break
                    print(f"❌ Please enter a number between 1 and {len(state['apriltag_detections'])}")

            # Ensure AprilTag definition exists in YAML
            tag_id = selected_apriltag['tag_id']
            self._ensure_apriltag_definition(f"tag_{tag_id}")

            # Store observation pose with equipment link
            position_data = {
                'coordinates': current_pose.tolist(),
                'joints': current_joints.round(3).tolist(),
                'description': description,
                'pose_type': 'observation',  # Mark as observation pose
                'tag_reference': f"tag_{tag_id}",
                'camera_to_tag': self._extract_camera_to_tag_transform(selected_apriltag),
                'equipment_name': equipment_name,  # MANDATORY equipment link
            }

            self.taught_positions['positions'][position_name] = position_data
            self.save_positions()
            print(f"✅ Observation pose '{position_name}' taught successfully!")
            print(f"⚙️  Linked to equipment: {equipment_name}")
            print(f"🏷️  Primary AprilTag: {tag_id}")

            # Disable freedrive mode after successful teaching
            try:
                self.disable_freedrive()
            except Exception as e:
                print(f"⚠️  Warning: Could not disable freedrive: {e}")

            return True

        except KeyboardInterrupt:
            print("\n⚠️  Observation pose teaching interrupted")
            # Try to disable freedrive on interruption
            try:
                self.disable_freedrive()
            except Exception as e:
                print(f"⚠️  Warning: Could not disable freedrive: {e}")
            return False
        except Exception as e:
            print(f"❌ Failed to teach observation pose: {e}")
            # Try to disable freedrive on error
            try:
                self.disable_freedrive()
            except Exception as e:
                print(f"⚠️  Warning: Could not disable freedrive: {e}")
            return False

    def _extract_camera_to_tag_transform(self, apriltag_data):
        """Extract camera-to-tag transform from AprilTag detection data"""
        if not apriltag_data:
            return [0, 0, 0, 0, 0, 0]  # Return zeros if no data

        # Extract pose data from AprilTag detection
        pose = apriltag_data.get('pose', {})
        if not pose or not pose.get('success', False):
            return [0, 0, 0, 0, 0, 0]  # Return zeros if no valid pose

        # Extract translation and rotation vectors
        translation = pose.get('translation_vector', [0, 0, 0])
        rotation = pose.get('rotation_vector', [0, 0, 0])

        # Ensure we have valid lists with 3 elements each
        if len(translation) != 3 or len(rotation) != 3:
            return [0, 0, 0, 0, 0, 0]

        # Return as [x, y, z, rx, ry, rz] array
        return translation + rotation

    def create_simple_safe_offset(self, base_position_name):
        """
        Create a simple safe offset position by asking for direction and distance.
        """
        try:
            # Check if base position exists
            if base_position_name not in self.taught_positions.get('positions', {}):
                print(f"❌ Base position '{base_position_name}' not found!")
                return False

            print(f"\n🛡️  Create safe offset position for '{base_position_name}'")

            # Ask for direction
            print("📐 Select offset direction:")
            print("  1) Up (Z+)")
            print("  2) Down (Z-)")
            print("  3) Back (Y-)")
            print("  4) Cancel")

            while True:
                try:
                    direction_choice = input("Enter choice (1-4): ").strip()
                    if direction_choice == '4':
                        print("❌ Safe offset creation cancelled")
                        return False
                    elif direction_choice in ['1', '2', '3']:
                        break
                    else:
                        print("❌ Please enter 1, 2, 3, or 4")
                except (EOFError, KeyboardInterrupt):
                    print("\n❌ Safe offset creation cancelled")
                    return False

            # Ask for distance
            while True:
                try:
                    distance_str = input("Enter distance in mm (e.g., 20): ").strip()
                    if not distance_str:
                        print("❌ Safe offset creation cancelled")
                        return False
                    distance_mm = float(distance_str)
                    if distance_mm <= 0:
                        print("❌ Distance must be positive")
                        continue
                    break
                except ValueError:
                    print("❌ Please enter a valid number")
                except (EOFError, KeyboardInterrupt):
                    print("\n❌ Safe offset creation cancelled")
                    return False

            # Convert mm to meters
            distance_m = distance_mm / 1000.0

            # Create offset based on direction
            offset = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # [x, y, z, rx, ry, rz]
            direction_names = {
                '1': ('Up', 'Z+'),
                '2': ('Down', 'Z-'),
                '3': ('Back', 'Y-')
            }

            if direction_choice == '1':      # Up
                offset[2] = distance_m
            elif direction_choice == '2':    # Down
                offset[2] = -distance_m
            elif direction_choice == '3':    # Back
                offset[1] = -distance_m

            direction_name, axis_name = direction_names[direction_choice]
            safe_position_name = f"{base_position_name}-safe"

            print(f"📊 Creating safe position: {direction_name} {distance_mm}mm ({axis_name})")

            # Check if safe position name already exists
            if safe_position_name in self.taught_positions.get('positions', {}):
                overwrite = input(f"⚠️  Position '{safe_position_name}' already exists. Overwrite? (y/N): ").lower().strip()
                if overwrite != 'y':
                    print("❌ Safe position creation cancelled")
                    return False

            # Get base position data and apply offset
            base_data = self.taught_positions['positions'][base_position_name]
            base_coords = base_data['coordinates']
            safe_coords = [base_coords[i] + offset[i] for i in range(6)]

            # Create safe position data (deep copy to avoid YAML reference issues)
            import copy
            safe_data = copy.deepcopy(base_data)
            safe_data['coordinates'] = safe_coords
            safe_data['description'] = f"Safe {direction_name.lower()} position ({distance_mm}mm from {base_position_name})"

            # Update observation_offset for the safe position (calculate new offset from observation pose)
            if safe_data.get('observation_pose'):
                obs_pose_name = safe_data['observation_pose']
                if obs_pose_name in self.taught_positions['positions']:
                    obs_coords = np.array(self.taught_positions['positions'][obs_pose_name]['coordinates'])
                    safe_coords_array = np.array(safe_coords)
                    new_offset = (safe_coords_array - obs_coords).tolist()
                    safe_data['observation_offset'] = new_offset
                else:
                    # If observation pose doesn't exist, use zero offset
                    safe_data['observation_offset'] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

            # Update camera-to-tag if it exists (adjust distance for offsets)
            if safe_data.get('camera_to_tag') and (offset[1] != 0 or offset[2] != 0):
                camera_to_tag = safe_data['camera_to_tag'].copy()
                if len(camera_to_tag) >= 3:
                    if offset[1] != 0:  # Y offset (back/forward)
                        camera_to_tag[1] += offset[1]
                    if offset[2] != 0:  # Z offset (up/down)
                        camera_to_tag[2] += offset[2]
                    safe_data['camera_to_tag'] = camera_to_tag

            # Store the safe position
            self.taught_positions['positions'][safe_position_name] = safe_data
            self.save_positions()

            print(f"✅ Safe position '{safe_position_name}' created successfully!")
            print(f"📍 Base: {[round(x, 3) for x in base_coords]}")
            print(f"🛡️  Safe: {[round(x, 3) for x in safe_coords]}")

            # Reachability check - test movement to the safe position
            print("\n🔍 Testing reachability of safe position...")
            reachability_test = input("Perform reachability test? (Y/n): ").lower().strip()

            if reachability_test != 'n' and reachability_test != 'no':
                if self.test_safe_position_reachability(safe_position_name):
                    print(f"✅ Safe position '{safe_position_name}' is reachable and verified!")
                else:
                    print(f"⚠️  Safe position '{safe_position_name}' may have reachability issues")
                    delete_unsafe = input("Delete unsafe position? (y/N): ").lower().strip()
                    if delete_unsafe == 'y':
                        del self.taught_positions['positions'][safe_position_name]
                        self.save_positions()
                        print(f"🗑️  Deleted unreachable safe position '{safe_position_name}'")
                        return False

            return True

        except Exception as e:
            print(f"❌ Error creating safe offset position: {e}")
            return False

    def test_safe_position_reachability(self, position_name):
        """
        Test if a safe position is reachable by attempting to move to it.

        Args:
            position_name: Name of the position to test

        Returns:
            bool: True if position is reachable, False otherwise
        """
        try:
            if position_name not in self.taught_positions.get('positions', {}):
                print(f"❌ Position '{position_name}' not found for reachability test")
                return False

            print(f"🔍 Testing reachability of '{position_name}'...")

            # Store current position for return
            if not self.connect_for_movement():
                print("❌ Cannot connect for movement test")
                return False

            try:
                initial_pose = self.robot.get_tcp_pose()
                # Convert numpy array to regular list for display
                initial_pose_list = [float(x) for x in initial_pose]
                print(f"📍 Saving current position: {[round(x, 3) for x in initial_pose_list]}")
            except Exception as e:
                print(f"⚠️  Could not get current position: {e}")
                initial_pose = None

            # Test movement to the safe position
            print(f"🤖 Attempting to move to safe position '{position_name}'...")

            try:
                # Use the existing move_to_position method (without updating usage count)
                position_data = self.taught_positions['positions'][position_name]
                target_pose = position_data['coordinates']

                # Convert list to numpy array for robot movement
                target_pose_array = np.array(target_pose, dtype=float)

                print(f"🎯 Target: {[round(x, 3) for x in target_pose]}")
                print("⏳ Moving robot... (this may take a few seconds)")

                success = self.robot.move_to_pose(target_pose_array)

                if success:
                    print("✅ Movement successful - safe position is reachable!")

                    # Brief pause at the safe position
                    time.sleep(1.0)

                    # Return to initial position if we have it
                    if initial_pose is not None:
                        print("🔄 Returning to initial position...")
                        # Ensure initial_pose is numpy array
                        if not isinstance(initial_pose, np.ndarray):
                            initial_pose = np.array(initial_pose, dtype=float)
                        return_success = self.robot.move_to_pose(initial_pose)

                        if return_success:
                            print("✅ Returned to initial position")
                        else:
                            print("⚠️  Could not return to initial position")

                    return True
                else:
                    print("❌ Movement failed - position may not be reachable")
                    return False

            except Exception as move_error:
                print(f"❌ Movement error: {move_error}")
                print("⚠️  Position may be outside robot workspace or have collision issues")

                # Try to return to initial position if possible
                if initial_pose is not None:
                    try:
                        print("🔄 Attempting to return to initial position...")
                        # Ensure initial_pose is numpy array
                        if not isinstance(initial_pose, np.ndarray):
                            initial_pose = np.array(initial_pose, dtype=float)
                        self.robot.move_to_pose(initial_pose)
                        print("✅ Returned to initial position after failed test")
                    except Exception as return_error:
                        print(f"⚠️  Could not return to initial position after failed test: {return_error}")

                return False

        except Exception as e:
            print(f"❌ Error during reachability test: {e}")
            return False

    def teach_safe_offset_position(self, base_position_name, safe_position_name, offset, description=""):
        """
        Teach a safe offset position based on an existing position.

        Args:
            base_position_name: Name of existing position to offset from
            safe_position_name: Name for the new safe position
            offset: List of [x, y, z, rx, ry, rz] offsets to apply
            description: Optional description for the safe position
        """
        try:
            # Check if base position exists
            if base_position_name not in self.taught_positions.get('positions', {}):
                print(f"❌ Base position '{base_position_name}' not found!")
                print("💡 Use 'list' command to see available positions")
                return False

            # Check if safe position name already exists
            if safe_position_name in self.taught_positions.get('positions', {}):
                overwrite = input(f"⚠️  Position '{safe_position_name}' already exists. Overwrite? (y/N): ").lower().strip()
                if overwrite != 'y':
                    print("❌ Safe position creation cancelled")
                    return False

            # Get base position data
            base_data = self.taught_positions['positions'][base_position_name]
            base_coords = base_data['coordinates']

            print(f"\n🏗️  Creating safe offset position from '{base_position_name}'")
            print(f"📍 Base coordinates: {base_coords}")
            print(f"📐 Applying offset: {offset}")

            # Apply offset to coordinates
            safe_coords = [base_coords[i] + offset[i] for i in range(6)]

            print(f"🎯 Safe coordinates: {safe_coords}")

            # Copy most data from base position but update key fields
            safe_data = base_data.copy()
            safe_data['coordinates'] = safe_coords
            safe_data['description'] = description or f"Safe offset position from {base_position_name}"

            # Update camera-to-tag if it exists (adjust Z distance for vertical offsets)
            if safe_data.get('camera_to_tag') and offset[2] != 0:  # Z offset
                camera_to_tag = safe_data['camera_to_tag'].copy()
                if len(camera_to_tag) >= 3:
                    camera_to_tag[2] += offset[2]  # Adjust Z distance in camera frame
                    safe_data['camera_to_tag'] = camera_to_tag

            # Store the safe position
            self.taught_positions['positions'][safe_position_name] = safe_data
            self.save_positions()

            print(f"✅ Safe offset position '{safe_position_name}' created successfully!")
            print(f"📊 Offset applied: X:{offset[0]:+.3f}, Y:{offset[1]:+.3f}, Z:{offset[2]:+.3f}, Rx:{offset[3]:+.3f}, Ry:{offset[4]:+.3f}, Rz:{offset[5]:+.3f}")

            return True

        except Exception as e:
            print(f"❌ Failed to create safe offset position: {e}")
            return False

    def _get_apriltag_positions(self):
        """Get list of position names that have direct AprilTag view"""
        apriltag_positions = []
        positions = self.taught_positions.get('positions', {})

        for name, data in positions.items():
            if data.get('tag_reference') and data.get('camera_to_tag'):
                apriltag_positions.append(name)

        return apriltag_positions

    def _validate_observation_pose(self, position_name, observation_pose):
        """Validate that observation pose exists and has AprilTag view"""
        if not observation_pose:
            return True  # No observation pose is valid

        positions = self.taught_positions.get('positions', {})

        # Check if observation pose exists
        if observation_pose not in positions:
            print(f"❌ Observation pose '{observation_pose}' does not exist")
            return False

        # Check if observation pose has AprilTag view
        obs_data = positions[observation_pose]
        if not (obs_data.get('tag_reference') and obs_data.get('camera_to_tag')):
            print(f"❌ Observation pose '{observation_pose}' does not have AprilTag view")
            return False

        # Check for circular reference
        obs_observation = obs_data.get('observation_pose')
        if obs_observation == position_name:
            print(f"❌ Circular reference: '{observation_pose}' cannot reference '{position_name}'")
            return False

        return True

    def _ensure_apriltag_definition(self, tag_reference):
        """Ensure AprilTag definition exists in apriltags section"""
        if 'apriltags' not in self.taught_positions:
            self.taught_positions['apriltags'] = {}

        apriltags = self.taught_positions['apriltags']

        if tag_reference not in apriltags:
            # Extract tag ID from reference (e.g., "tag_1" -> 1)
            tag_id = tag_reference.replace('tag_', '')

            # Add default AprilTag definition
            apriltags[tag_reference] = {
                'dictionary': "DICT_4X4_1000",
                'marker_length': 0.023,  # Default size in meters
                'description': f"AprilTag {tag_id} marker"
            }

            print(f"➕ Added AprilTag definition: {tag_reference}")

        return apriltags[tag_reference]

    def teach_associated_view(self, parent_position_name, observation_name):
        """Teach an observation pose associated with a work pose using manual freedrive"""
        print(f"\n🔍 Teaching observation pose: {observation_name}")
        print(f"🔗 Associated with work pose: {parent_position_name}")
        print("🎯 Move robot to position where camera can see the relevant AprilTag")

        try:
            print("\n" + "=" * 60)
            print("🔍 OBSERVATION POSE TEACHING")
            print("=" * 60)
            print("📍 Use teach pendant to move robot so camera can see the AprilTag marker")
            print("🏷️  This pose will be used to verify the work position")
            print("⚠️  Make sure the robot is in local control mode!")
            print("=" * 60)

            # Wait for user to position robot manually
            input("Press ENTER when AprilTag is visible in camera view...")

            # Get current robot state from RTDE
            current_pose = self.robot.get_tcp_pose()
            current_joints = self.robot.get_joint_positions()

            print("\n📊 Current robot position:")
            print(f"   TCP Pose: {self.robot.format_pose(current_pose)}")
            print(f"   Joints: {current_joints.round(3)} rad")

            print("\n📷 Capturing scene for AprilTag detection...")

            # Capture current state
            state = self.capture_current_state()
            state['robot_pose'] = current_pose
            state['joint_positions'] = current_joints

            # Check for AprilTag detection
            if not state['apriltag_detections']:
                print("❌ No AprilTags detected in observation pose")
                response = input("Save observation pose without AprilTag? (y/N): ")
                if response.lower() != 'y':
                    print("❌ Observation pose teaching cancelled")
                    return False
            else:
                print(f"✅ Detected {len(state['apriltag_detections'])} AprilTag(s):")

                # Display all detected tags with indices
                for i, det in enumerate(state['apriltag_detections'], 1):
                    tag_id = det['tag_id']
                    distance = det.get('distance_mm', 'unknown')
                    print(f"   {i}. Tag ID {tag_id}: {distance}mm away")

                # Select which AprilTag to associate with this observation pose
                selected_apriltag = None
                if len(state['apriltag_detections']) == 1:
                    # Only one tag - use it automatically
                    selected_apriltag = state['apriltag_detections'][0]
                    print(f"✅ Using Tag ID {selected_apriltag['tag_id']} (only tag detected)")
                else:
                    # Multiple tags - ask user to choose
                    while True:
                        choice = input(f"Select AprilTag for observation pose (1-{len(state['apriltag_detections'])}): ").strip()
                        if choice.isdigit():
                            idx = int(choice) - 1
                            if 0 <= idx < len(state['apriltag_detections']):
                                selected_apriltag = state['apriltag_detections'][idx]
                                print(f"✅ Selected Tag ID {selected_apriltag['tag_id']}")
                                break
                        print(f"❌ Please enter a number between 1 and {len(state['apriltag_detections'])}")

                # Ensure AprilTag definition exists in YAML
                tag_id = selected_apriltag['tag_id']
                self._ensure_apriltag_definition(f"tag_{tag_id}")

            # Confirm save
            confirm = input(f"\n💾 Save observation pose '{observation_name}'? (y/N): ").lower().strip()
            if confirm != 'y':
                print("❌ Observation pose teaching cancelled")
                return False

            # Store observation pose
            observation_data = {
                'name': observation_name,
                'description': f"Observation pose for {parent_position_name}",
                'robot_pose': current_pose,
                'joint_positions': current_joints,
                'apriltag_detections': state['apriltag_detections'],
                'image_path': state['image_path'],
                'taught_timestamp': state['timestamp'],
                'teaching_method': 'manual',
                'parent_position': parent_position_name  # Link to work pose
            }

            self.taught_positions[observation_name] = observation_data
            self.save_positions()

            print(f"✅ Observation pose '{observation_name}' taught successfully")
            print(f"🔗 Linked to work pose '{parent_position_name}'")
            return True

        except KeyboardInterrupt:
            print("\n⚠️  Observation pose teaching interrupted")
            return False
        except Exception as e:
            print(f"❌ Failed to teach observation pose: {e}")
            return False

    def link_observation_pose(self, position_name, observation_pose_name):
        """Link a position to an observation pose for AprilTag correction"""
        positions = self.taught_positions.get('positions', {})

        # Validate position exists
        if position_name not in positions:
            print(f"❌ Position '{position_name}' not found")
            return False

        # Validate observation pose exists
        if observation_pose_name not in positions:
            print(f"❌ Observation pose '{observation_pose_name}' not found")
            return False

        # Validate observation pose has AprilTag view
        obs_pose = positions[observation_pose_name]
        if not (obs_pose.get('tag_reference') and obs_pose.get('camera_to_tag')):
            print(f"❌ '{observation_pose_name}' is not a valid observation pose (no AprilTag view)")
            return False

        # Check if position already has direct AprilTag view
        position = positions[position_name]
        if position.get('tag_reference') and position.get('camera_to_tag'):
            print(f"⚠️  Position '{position_name}' already has direct AprilTag view")
            confirm = input("Link to observation pose anyway? (y/N): ").lower().strip()
            if confirm != 'y':
                print("❌ Linking cancelled")
                return False

        # Update the position's observation_pose field
        self.taught_positions['positions'][position_name]['observation_pose'] = observation_pose_name
        self.save_positions()

        print(f"✅ Linked '{position_name}' to observation pose '{observation_pose_name}'")
        print(f"🔗 AprilTag correction will use: {obs_pose.get('tag_reference', 'unknown tag')}")
        return True

    def list_positions(self):
        """List all taught positions"""
        positions = self.taught_positions.get('positions', {})
        if not positions:
            print("📭 No positions have been taught yet")
            return

        print(f"\n📋 Taught Positions ({len(positions)})")
        print("=" * 60)

        for name, data in positions.items():
            desc = data.get('description', 'No description')
            timestamp = data.get('timestamp', 'Unknown time')
            has_apriltag = bool(data.get('tag_reference') and data.get('camera_to_tag'))
            tag_reference = data.get('tag_reference')
            observation_pose = data.get('observation_pose')
            equipment_name = data.get('equipment_name')
            pose_type = data.get('pose_type', 'work')

            print(f"🎯 {name}")
            if pose_type == 'observation':
                print("   🔍 Type: Observation Pose")
                if equipment_name:
                    print(f"   ⚙️  Equipment: {equipment_name}")
            else:
                print("   📍 Type: Work Position")
            print(f"   📝 {desc}")

            if has_apriltag:
                tag_info = f"Tag {tag_reference}" if tag_reference else "detected"
                print(f"   👁️  AprilTag View: ✅ Direct ({tag_info})")
            elif observation_pose:
                print(f"   👁️  AprilTag View: ↗️  Via observation pose '{observation_pose}'")
            else:
                print("   👁️  AprilTag View: ❌ None (no correction)")

            print(f"   📅 Taught: {timestamp}")
            print()

    def move_to_position(self, position_name):
        """Move robot to a taught position"""
        if position_name not in self.taught_positions.get('positions', {}):
            print(f"❌ Position '{position_name}' not found")
            return False

        position_data = self.taught_positions['positions'][position_name]
        target_pose = np.array(position_data['coordinates'])

        print(f"🎯 Moving to position: {position_name}")
        print(f"📍 Target pose: {self.robot.format_pose(target_pose)}")

        # Ensure we have movement control
        if not self.connect_for_movement():
            return False

        try:
            # Move to position
            success = self.robot.move_to_pose(target_pose)

            if success:
                print(f"✅ Successfully moved to position '{position_name}'")
                return True
            else:
                print(f"❌ Failed to move to position '{position_name}'")
                return False

        except Exception as e:
            print(f"❌ Movement error: {e}")
            return False

    def verify_position(self, position_name):
        """Verify a taught position by checking AprilTag visibility"""
        if position_name not in self.taught_positions.get('positions', {}):
            print(f"❌ Position '{position_name}' not found")
            return False

        position_data = self.taught_positions['positions'][position_name]
        expected_tags = [det['tag_id'] for det in position_data.get('apriltag_detections', [])]

        if not expected_tags:
            print(f"⚠️  Position '{position_name}' has no associated AprilTags to verify")
            return True

        print(f"🔍 Verifying position: {position_name}")
        print(f"🏷️  Expected AprilTags: {expected_tags}")

        try:
            # Capture current view
            photo_path = self.camera.capture_photo()
            image = cv2.imread(photo_path)
            detections = self.detector.detect_tags(image)

            detected_tags = [det['tag_id'] for det in detections]
            print(f"👁️  Currently visible tags: {detected_tags}")

            # Check if expected tags are visible
            missing_tags = set(expected_tags) - set(detected_tags)
            extra_tags = set(detected_tags) - set(expected_tags)

            if not missing_tags and not extra_tags:
                print("✅ Position verification successful - all expected tags visible")
                return True
            else:
                if missing_tags:
                    print(f"⚠️  Missing expected tags: {list(missing_tags)}")
                if extra_tags:
                    print(f"ℹ️  Additional tags visible: {list(extra_tags)}")
                return False

        except Exception as e:
            print(f"❌ Verification error: {e}")
            return False

    def delete_position(self, position_name):
        """Delete a taught position"""
        if position_name not in self.taught_positions.get('positions', {}):
            print(f"❌ Position '{position_name}' not found")
            return False

        print(f"🗑️  Deleting position: {position_name}")
        response = input("Are you sure? This cannot be undone (y/N): ")

        if response.lower() == 'y':
            del self.taught_positions['positions'][position_name]
            self.save_positions()
            print(f"✅ Position '{position_name}' deleted")
            return True
        else:
            print("❌ Deletion cancelled")
            return False


def main():
    parser = argparse.ArgumentParser(description='Robot Position Teaching CLI')
    parser.add_argument('--robot-ip', help='Robot IP address (overrides config)')
    parser.add_argument('--camera-host', help='Camera host (overrides config)')
    parser.add_argument('--camera-port', type=int, help='Camera port (overrides config)')
    parser.add_argument('--prefix', default='', help='Prefix to add to position names (e.g., "station1-", "robot2-")')

    subparsers = parser.add_subparsers(dest='command', help='Available commands')

    # Teach position command
    teach_parser = subparsers.add_parser('teach', help='Teach a new position')
    teach_parser.add_argument('name', help='Position name')
    teach_parser.add_argument('--description', default='', help='Position description')

    # Teach observation pose command
    observe_parser = subparsers.add_parser('observe', help='Teach an observation pose for equipment')
    observe_parser.add_argument('name', help='Observation pose name')
    observe_parser.add_argument('equipment', help='Equipment name (mandatory)')
    observe_parser.add_argument('--description', default='', help='Observation pose description')

    # Teach safe offset position command
    safe_parser = subparsers.add_parser('safe', help='Teach a safe offset position based on existing position')
    safe_parser.add_argument('base_position', help='Base position name to offset from')
    safe_parser.add_argument('safe_name', help='Safe position name (will be created)')
    safe_parser.add_argument('--offset', nargs=6, type=float, default=[0, 0, 0.02, 0, 0, 0],
                             help='Offset [x, y, z, rx, ry, rz] (default: [0, 0, 0.02, 0, 0, 0] for 2cm up)')
    safe_parser.add_argument('--description', default='', help='Safe position description')

    # Link observation pose command
    link_parser = subparsers.add_parser('link', help='Link a position to an observation pose')
    link_parser.add_argument('position', help='Position name to link')
    link_parser.add_argument('observation_pose', help='Observation pose name')

    # List positions command
    subparsers.add_parser('list', help='List all taught positions')

    # Move to position command
    move_parser = subparsers.add_parser('move', help='Move to a taught position')
    move_parser.add_argument('name', help='Position name')

    # Verify position command
    verify_parser = subparsers.add_parser('verify', help='Verify position by checking AprilTag visibility')
    verify_parser.add_argument('name', help='Position name')

    # Delete position command
    delete_parser = subparsers.add_parser('delete', help='Delete a taught position')
    delete_parser.add_argument('name', help='Position name')

    # Interactive mode
    subparsers.add_parser('interactive', help='Interactive teaching mode')

    args = parser.parse_args()

    if not args.command:
        parser.print_help()
        return

    # Initialize position teacher
    teacher = PositionTeacher(
        robot_ip=args.robot_ip,
        camera_host=args.camera_host,
        camera_port=args.camera_port
    )

    try:
        # Helper function to apply prefix to position names
        def apply_prefix(name):
            if args.prefix and not name.startswith(args.prefix):
                return f"{args.prefix}{name}"
            return name

        if args.command in ['teach', 'observe', 'move', 'verify']:
            # Commands that require connections
            if not teacher.connect():
                return

        # Execute commands
        if args.command == 'teach':
            prefixed_name = apply_prefix(args.name)
            teacher.teach_position(prefixed_name, args.description)
        elif args.command == 'observe':
            prefixed_name = apply_prefix(args.name)
            teacher.teach_observation_pose(prefixed_name, args.equipment, args.description)
        elif args.command == 'safe':
            prefixed_base = apply_prefix(args.base_position)
            prefixed_safe = apply_prefix(args.safe_name)
            teacher.teach_safe_offset_position(prefixed_base, prefixed_safe, args.offset, args.description)
        elif args.command == 'link':
            prefixed_position = apply_prefix(args.position)
            prefixed_observation = apply_prefix(args.observation_pose)
            teacher.link_observation_pose(prefixed_position, prefixed_observation)
        elif args.command == 'list':
            teacher.list_positions()
        elif args.command == 'move':
            prefixed_name = apply_prefix(args.name)
            teacher.move_to_position(prefixed_name)
        elif args.command == 'verify':
            prefixed_name = apply_prefix(args.name)
            teacher.verify_position(prefixed_name)
        elif args.command == 'delete':
            prefixed_name = apply_prefix(args.name)
            teacher.delete_position(prefixed_name)
        elif args.command == 'interactive':
            interactive_mode(teacher, args.prefix)

    finally:
        teacher.disconnect()


def interactive_mode(teacher, prefix=''):
    """Interactive position teaching mode with menu interface"""
    if not teacher.connect():
        return

    # Helper function to apply prefix
    def apply_prefix(name):
        if prefix and not name.startswith(prefix):
            return f"{prefix}{name}"
        return name

    def display_menu():
        print("\n" + "=" * 60)
        print("🎮 ROBOT POSITION TEACHING SYSTEM")
        if prefix:
            print(f"🏷️  Using prefix: '{prefix}'")
        print("=" * 60)
        print("1️⃣  Teach Position with Remote Freedrive ⭐ [teach]")
        print("2️⃣  Teach Observation Pose [observe]")
        print("3️⃣  Create Safe Offset Position [safe]")
        print("4️⃣  Link Position to Observation Pose [link]")
        print("5️⃣  List All Positions [list]")
        print("6️⃣  Move to Position [move]")
        print("7️⃣  Verify Position (AprilTag check) [verify]")
        print("8️⃣  Delete Position [delete]")
        print("0️⃣  Exit [exit]")
        print("=" * 60)
        print("💡 Enter number (1-8) or keyword [teach, observe, safe, link, list, move, verify, delete, exit]")

    def get_input(prompt, required=True):
        while True:
            value = input(prompt).strip()
            if value or not required:
                return value
            print("❌ This field is required. Please enter a value.")

    def get_offset():
        print("📐 Enter offset values (or press Enter for default: 0 0 0.02 0 0 0)")
        offset_str = input("   Format: x y z rx ry rz: ").strip()
        if not offset_str:
            return [0, 0, 0.02, 0, 0, 0]  # Default: 2cm up
        try:
            return [float(x) for x in offset_str.split()]
        except ValueError:
            print("❌ Invalid offset values. Using default.")
            return [0, 0, 0.02, 0, 0, 0]

    def parse_menu_choice(choice):
        """Parse menu choice - accepts both numbers and keywords"""
        choice = choice.lower().strip()

        # Direct number mapping
        if choice in ['1', '2', '3', '4', '5', '6', '7', '8', '0']:
            return choice

        # Keyword mapping
        keyword_map = {
            'teach': '1',
            't': '1',
            'observe': '2',
            'obs': '2',
            'o': '2',
            'safe': '3',
            's': '3',
            'link': '4',
            'l': '4',
            'list': '5',
            'ls': '5',
            'move': '6',
            'm': '6',
            'verify': '7',
            'v': '7',
            'delete': '8',
            'del': '8',
            'd': '8',
            'exit': '0',
            'quit': '0',
            'q': '0'
        }

        return keyword_map.get(choice, choice)  # Return original if no match

    print("\n🎮 Interactive Position Teaching Mode")
    print("🤲 Use freedrive mode on teach pendant to position robot")

    while True:
        try:
            display_menu()
            user_input = input("Select option: ").strip()
            choice = parse_menu_choice(user_input)

            if choice == '0':
                print("👋 Exiting interactive mode...")
                break

            elif choice == '1':
                print("\n🎯 TEACH POSITION WITH REMOTE FREEDRIVE ⭐")
                print("🔄 Robot stays in Remote control mode")
                name = get_input("Position name: ")
                name = apply_prefix(name)
                description = get_input("Description (optional): ", required=False)
                teacher.teach_position_with_freedrive(name, description)

            elif choice == '2':
                print("\n👁️  TEACH OBSERVATION POSE")
                name = get_input("Observation pose name: ")
                name = apply_prefix(name)
                equipment = get_input("Equipment name: ")
                description = get_input("Description (optional): ", required=False)
                teacher.teach_observation_pose(name, equipment, description)

            elif choice == '3':
                print("\n🛡️  CREATE SAFE OFFSET POSITION")
                teacher.list_positions()
                base_position = get_input("Base position name: ")
                base_position = apply_prefix(base_position)
                safe_name = get_input("Safe position name: ")
                safe_name = apply_prefix(safe_name)
                offset = get_offset()
                description = get_input("Description (optional): ", required=False)
                teacher.teach_safe_offset_position(base_position, safe_name, offset, description)

            elif choice == '4':
                print("\n🔗 LINK POSITION TO OBSERVATION POSE")
                teacher.list_positions()
                position = get_input("Position name: ")
                position = apply_prefix(position)
                observation_pose = get_input("Observation pose name: ")
                observation_pose = apply_prefix(observation_pose)
                teacher.link_observation_pose(position, observation_pose)

            elif choice == '5':
                print("\n📋 ALL TAUGHT POSITIONS")
                teacher.list_positions()

            elif choice == '6':
                print("\n🎯 MOVE TO POSITION")
                teacher.list_positions()
                name = get_input("Position name to move to: ")
                name = apply_prefix(name)
                teacher.move_to_position(name)

            elif choice == '7':
                print("\n🔍 VERIFY POSITION")
                teacher.list_positions()
                name = get_input("Position name to verify: ")
                name = apply_prefix(name)
                teacher.verify_position(name)

            elif choice == '8':
                print("\n🗑️  DELETE POSITION")
                teacher.list_positions()
                name = get_input("Position name to delete: ")
                name = apply_prefix(name)
                teacher.delete_position(name)

            else:
                print(f"❌ Invalid choice: '{user_input}'. Please enter a number (0-8) or keyword (teach, list, etc.).")

        except KeyboardInterrupt:
            print("\n👋 Exiting interactive mode...")
            break
        except Exception as e:
            print(f"❌ Error: {e}")

        # Pause before showing menu again
        if choice != '0':
            input("\nPress Enter to continue...")


if __name__ == "__main__":
    main()
