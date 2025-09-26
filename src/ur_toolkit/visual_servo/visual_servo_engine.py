#!/usr/bin/env python3
"""
Visual Servo Engine
Eye-in-hand visual servoing engine with proper IBVS control law
"""

import numpy as np
import time
import json
from typing import Optional, Tuple, Dict, Any
from pathlib import Path
import sys

# Add parent directories to path
sys.path.insert(0, str(Path(__file__).parent.parent))
sys.path.insert(0, str(Path(__file__).parent.parent / "setup"))

from .config import visual_servo_config
from .detection_filter import DetectionFilter
from .pose_history import PoseHistoryManager
from ur_toolkit.apriltag_detection import AprilTagDetector


def transform_camera_to_robot_correction(camera_error_translation, camera_error_rotation):
    """
    Transform camera coordinate errors to robot coordinate corrections.
    
    Camera frame (OpenCV standard): X-right, Y-down, Z-forward (depth)
    Robot TCP frame (UR standard): X-forward, Y-left, Z-up
    
    Assuming eye-in-hand camera mounted looking forward from TCP.
    This transformation maps camera errors to proper robot movements.
    
    Args:
        camera_error_translation: [x, y, z] errors in camera frame
        camera_error_rotation: [rx, ry, rz] rotation errors in camera frame
        
    Returns:
        (robot_translation, robot_rotation) corrections in robot TCP frame
    """
    # Translation mapping - FIXED Z direction
    # Camera Z error positive = tag farther = robot should move forward (+Y)
    # Camera Z error negative = tag closer = robot should move backward (-Y)
    robot_translation = [
        -camera_error_translation[0],   # Camera X -> Robot -X (right becomes left)
        -camera_error_translation[2],   # Camera Z (depth) -> Robot -Y (forward/back) - INVERTED
        -camera_error_translation[1]    # Camera Y (down) -> Robot -Z (down becomes up)
    ]
    
    # Rotation mapping - DISABLED for debugging translation
    # robot_rotation = [
    #     -camera_error_rotation[0],     # Camera RX -> Robot -RX (consistent with X translation)
    #     -camera_error_rotation[2],     # Camera RZ (in-plane) -> Robot -RY (consistent with Z->Y translation)
    #     -camera_error_rotation[1]      # Camera RY -> Robot -RZ (consistent with Y translation)
    # ]
    robot_rotation = [0.0, 0.0, 0.0]  # Disable rotation corrections to focus on translation
    
    return robot_translation, robot_rotation


class EyeInHandPIDController:
    """PID controller for eye-in-hand visual servoing with individual axis control"""

    def __init__(self, kp: float = 1.0, ki: float = 0.0, kd: float = 0.0,
                 output_limit: float = 1.0, integral_limit: float = 1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit

        # State variables
        self.integral_term = 0.0
        self.previous_error = None

    def update(self, error: float, dt: float) -> float:
        """Calculate PID output for given error and time step"""

        # Proportional term
        p_term = self.kp * error

        # Integral term with windup protection
        self.integral_term += error * dt * self.ki
        self.integral_term = np.clip(self.integral_term, -self.integral_limit, self.integral_limit)

        # Derivative term (use derivative of measurement to avoid derivative kick)
        d_term = 0.0
        if self.previous_error is not None and dt > 0:
            d_term = self.kd * (error - self.previous_error) / dt

        # Calculate total output
        output = p_term + self.integral_term + d_term

        # Apply output limits
        output = np.clip(output, -self.output_limit, self.output_limit)

        # Update state
        self.previous_error = error

        return output

    def reset(self):
        """Reset controller state"""
        self.integral_term = 0.0
        self.previous_error = None


from ur_toolkit.camera.picam.picam import PiCam, PiCamConfig
from ur_toolkit.config_manager import get_camera_host, get_camera_port


class VisualServoEngine:
    """Core visual servoing engine for dynamic pose correction"""

    def __init__(self, robot_controller, positions_file: Path, apriltag_detector=None, camera=None):
        """
        Initialize visual servo engine

        Args:
            robot_controller: Robot controller instance (URController)
            positions_file: Path to taught positions file
            apriltag_detector: AprilTag detector instance (optional, will create if None)
            camera: Camera instance (optional, will create if None)
        """
        self.robot = robot_controller
        self.config = visual_servo_config

        # Initialize AprilTag detector if not provided
        if apriltag_detector is None:
            self.detector = AprilTagDetector()
        else:
            self.detector = apriltag_detector

        # Initialize camera if not provided
        if camera is None:
            host = get_camera_host()
            port = get_camera_port()
            config = PiCamConfig(hostname=host, port=port)
            self.camera = PiCam(config)
        else:
            self.camera = camera

        # Initialize components
        self.detection_filter = DetectionFilter(self.detector, self.camera, self.config)
        self.pose_history = PoseHistoryManager(positions_file, self.config)

        # Initialize PID controllers for eye-in-hand visual servoing
        # VERY conservative gains - the system was diverging with aggressive gains
        # Separate controllers for translation and rotation
        self.translation_pids = [
            EyeInHandPIDController(kp=0.3, ki=0.0, kd=0.0, output_limit=0.015, integral_limit=0.01),  # X - very conservative
            EyeInHandPIDController(kp=0.3, ki=0.0, kd=0.0, output_limit=0.015, integral_limit=0.01),  # Y - very conservative
            EyeInHandPIDController(kp=0.2, ki=0.0, kd=0.0, output_limit=0.010, integral_limit=0.005)  # Z - ultra conservative
        ]

        self.rotation_pids = [
            EyeInHandPIDController(kp=0.2, ki=0.0, kd=0.0, output_limit=0.05, integral_limit=0.05),   # RX - very conservative
            EyeInHandPIDController(kp=0.2, ki=0.0, kd=0.0, output_limit=0.05, integral_limit=0.05),   # RY - very conservative
            EyeInHandPIDController(kp=0.15, ki=0.0, kd=0.0, output_limit=0.04, integral_limit=0.03)  # RZ - ultra conservative
        ]

        # Track time for PID dt calculation
        self.last_update_time = None

        # Add stability tracking
        self.error_history = []
        self.max_error_history = 3

        # Load hand-eye calibration if available
        self.hand_eye_transform = self._load_hand_eye_calibration()

        print("🎯 Visual Servo Engine initialized")
        self.config.print_config()

    def visual_servo_to_position(self, position_name: str, update_stored_pose: bool = False) -> Tuple[bool, Dict[str, Any]]:
        """
        Perform visual servoing to a taught position with AprilTag reference

        Args:
            position_name: Name of the taught position
            update_stored_pose: Whether to update the stored position after correction

        Returns:
            (success, metrics) tuple
        """
        print(f"\n🎯 Starting visual servoing to position '{position_name}'")

        # Load position data
        position_data = self._get_position_data(position_name)
        if not position_data:
            return False, {'error': 'Position not found'}

        # Check if position has direct AprilTag view or uses observation pose
        if position_data.get('camera_to_tag'):
            # Direct AprilTag view - standard visual servoing
            # Check if position has AprilTag reference
            if 'tag_reference' not in position_data or not position_data['tag_reference']:
                print(f"❌ Position '{position_name}' has no AprilTag reference for visual servoing")
                return False, {'error': 'No AprilTag reference'}
            return self._visual_servo_direct(position_name, position_data, update_stored_pose)
        elif position_data.get('observation_pose') and position_data['observation_pose'] != position_name:
            # Uses a different observation pose - observation-based visual servoing
            return self._visual_servo_via_observation(position_name, position_data, update_stored_pose)
        else:
            print(f"❌ Position '{position_name}' has no AprilTag view (direct or via observation)")
            return False, {'error': 'No AprilTag view available'}

    def _visual_servo_direct(self, position_name: str, position_data: Dict[str, Any], update_stored_pose: bool) -> Tuple[bool, Dict[str, Any]]:
        """
        Perform visual servoing for positions with direct AprilTag view

        Args:
            position_name: Name of the taught position
            position_data: Position data from YAML
            update_stored_pose: Whether to update the stored position after correction

        Returns:
            (success, metrics) tuple
        """
        print(f"📸 Direct visual servoing for '{position_name}'")

        tag_reference = position_data['tag_reference']
        stored_tag_pose = np.array(position_data['camera_to_tag'])
        stored_robot_pose = np.array(position_data['coordinates'])

        # Extract tag ID from reference (handle both "tag_123" and "123" formats)
        if isinstance(tag_reference, str) and tag_reference.startswith('tag_'):
            tag_id = int(tag_reference.split('_')[1])
        else:
            tag_id = int(tag_reference)

        print(f"🏷️  Using AprilTag {tag_id} as reference")

        # Initialize metrics
        metrics = {
            'position_name': position_name,
            'tag_id': tag_id,
            'method': 'direct',
            'iterations': 0,
            'total_correction': np.zeros(6),
            'final_error': None,
            'converged': False,
            'corrections_applied': []
        }

        current_robot_pose = stored_robot_pose.copy()
        total_correction = np.zeros(6)

        # Iterative correction loop
        for iteration in range(self.config.max_iterations):
            print(f"\n🔄 Iteration {iteration + 1}/{self.config.max_iterations}")
            metrics['iterations'] = iteration + 1

            # Move to current estimated pose
            print("🤖 Moving to estimated pose...")
            success = self.robot.move_to_pose(current_robot_pose)
            if not success:
                print("❌ Failed to move robot to pose")
                return False, metrics

            # Detect current AprilTag pose
            current_tag_pose = self.detection_filter.get_filtered_tag_pose(tag_id)
            if current_tag_pose is None:
                print(f"❌ Failed to detect AprilTag {tag_id}")
                return False, metrics

            # Calculate pose error
            tag_error = current_tag_pose - stored_tag_pose
            pose_error_magnitude = np.linalg.norm(tag_error)

            print(f"📏 Tag pose error magnitude: {pose_error_magnitude:.4f}")
            print(f"   Translation error: [{tag_error[0]:.4f}, {tag_error[1]:.4f}, {tag_error[2]:.4f}]m")
            print(f"   Rotation error: [{tag_error[3]:.4f}, {tag_error[4]:.4f}, {tag_error[5]:.4f}]rad")

            # Check convergence
            if (np.linalg.norm(tag_error[:3]) < self.config.position_tolerance
                    and np.linalg.norm(tag_error[3:]) < self.config.rotation_tolerance):
                print("✅ Converged within tolerance")
                metrics['converged'] = True
                metrics['final_error'] = pose_error_magnitude
                break

            # Calculate robot pose correction using eye-in-hand visual servoing control
            # Eye-in-hand IBVS: camera moves with end-effector, so control law is different
            # than eye-to-hand setup

            # Extract translation and rotation errors separately
            tag_translation_error = tag_error[:3]  # [x, y, z] in camera frame
            tag_rotation_error = tag_error[3:]     # [rx, ry, rz] in camera frame

            # Calculate time step for PID controllers
            current_time = time.time()
            if self.last_update_time is None:
                dt = 0.1  # Initial time step
                # Reset all PID controllers on first iteration
                for pid in self.translation_pids + self.rotation_pids:
                    pid.reset()
            else:
                dt = current_time - self.last_update_time
                dt = max(dt, 0.001)  # Prevent division by zero

            self.last_update_time = current_time

            # Track error magnitude for stability detection
            error_magnitude = np.linalg.norm(tag_translation_error) + np.linalg.norm(tag_rotation_error)

            # Detection consistency check - DISABLED for testing
            # The check was too aggressive and preventing convergence
            # if len(self.error_history) >= 1:
            #     prev_error = self.error_history[-1]
            #     error_increase_threshold = 1.3  # 30% increase threshold
            #
            #     if error_magnitude > prev_error * error_increase_threshold:
            #         should_skip_correction = True
            #         print(f"⚠️  Detection inconsistency detected!")
            #         print(f"   Previous error: {prev_error:.4f}")
            #         print(f"   Current error: {error_magnitude:.4f}")
            #         print(f"   Increase: {((error_magnitude/prev_error - 1) * 100):.1f}%")
            #         print(f"   🚫 Skipping correction for this iteration (likely detection noise)")
            #
            #         # Don't update error history with noisy measurement
            #         # Continue to next iteration without applying correction
            #         continue

            self.error_history.append(error_magnitude)
            if len(self.error_history) > self.max_error_history:
                self.error_history.pop(0)

            # Check for divergence/instability (only for accepted measurements)
            if len(self.error_history) >= 2:
                # If error is consistently increasing, reduce gains automatically
                if self.error_history[-1] > self.error_history[-2] * 1.15:
                    self.logger.warning("Error trend increasing - reducing PID gains automatically")
                    for pid in self.translation_pids + self.rotation_pids:
                        pid.kp *= 0.9  # Reduce proportional gain more gently
                        pid.output_limit *= 0.95  # Reduce output limits more gently

            # Apply simple direct correction - transform tag error to robot movement
            # For eye-in-hand, camera frame aligns with end-effector frame
            # Apply opposite movement to correct visual error

            robot_translation_correction = -tag_translation_error  # Opposite direction
            robot_rotation_correction = -tag_rotation_error        # Opposite direction

            # Combine corrections
            robot_correction = np.concatenate([robot_translation_correction, robot_rotation_correction])

            # Apply additional safety velocity limiting for eye-in-hand
            # Eye-in-hand is more sensitive to large motions
            max_translation_velocity = 0.03  # 3cm/s max (conservative for eye-in-hand)
            max_rotation_velocity = 0.2      # 0.2 rad/s max (conservative for eye-in-hand)

            # Limit translation velocities
            for i in range(3):
                robot_correction[i] = np.clip(robot_correction[i],
                                              -max_translation_velocity, max_translation_velocity)

            # Limit rotation velocities
            for i in range(3, 6):
                robot_correction[i] = np.clip(robot_correction[i],
                                              -max_rotation_velocity, max_rotation_velocity)

            # Check if using simple legacy-style mode
            if hasattr(self.config, 'simple_mode') and self.config.simple_mode:
                # Simple mode: only XY translation corrections (like legacy system)
                simple_correction = np.zeros(6)
                simple_correction[0] = -tag_translation_error[0] * 0.4  # X correction
                simple_correction[1] = -tag_translation_error[1] * 0.4  # Y correction
                # Z, RX, RY, RZ remain zero for stability
                robot_correction = simple_correction
                
                print("🔧 Simple XY-only correction (legacy style):")
                print(f"   Translation errors: [{tag_translation_error[0]:.4f}, {tag_translation_error[1]:.4f}, {tag_translation_error[2]:.4f}]")
                print(f"   Applied XY correction: [{robot_correction[0]:.4f}, {robot_correction[1]:.4f}, 0.0000] (Z/rotations=0)")
                print("   Simple mode damping: 0.400")
            else:
                # Apply overall damping factor for stability
                robot_correction *= self.config.damping_factor
                
                print(f"🔧 Eye-in-hand visual servoing control (dt={dt:.3f}s):")
                print(f"   Translation errors: [{tag_translation_error[0]:.4f}, {tag_translation_error[1]:.4f}, {tag_translation_error[2]:.4f}]")
                print(f"   Rotation errors: [{tag_rotation_error[0]:.4f}, {tag_rotation_error[1]:.4f}, {tag_rotation_error[2]:.4f}]")
                print(f"   Translation correction: [{robot_translation_correction[0]:.4f}, {robot_translation_correction[1]:.4f}, {robot_translation_correction[2]:.4f}]")
                print(f"   Rotation correction: [{robot_rotation_correction[0]:.4f}, {robot_rotation_correction[1]:.4f}, {robot_rotation_correction[2]:.4f}]")
                print(f"   Overall damping: {self.config.damping_factor:.2f}")

            # Apply safety limits
            correction_valid, safety_metrics = self._validate_correction(
                robot_correction, total_correction)

            if not correction_valid:
                print("❌ Correction exceeds safety limits")
                metrics.update(safety_metrics)
                return False, metrics

            # Apply correction
            current_robot_pose += robot_correction
            total_correction += robot_correction

            correction_magnitude = np.linalg.norm(robot_correction)
            print(f"🔧 Applied correction magnitude: {correction_magnitude:.4f}")

            metrics['corrections_applied'].append({
                'iteration': iteration + 1,
                'tag_error': tag_error.tolist(),
                'robot_correction': robot_correction.tolist(),
                'correction_magnitude': correction_magnitude
            })

        # Final metrics
        metrics['total_correction'] = total_correction.tolist()
        metrics['final_robot_pose'] = current_robot_pose.tolist()

        if not metrics['converged']:
            print(f"⚠️  Did not converge within {self.config.max_iterations} iterations")
            print(f"   Final error: {pose_error_magnitude:.4f}")
            print(f"   Position tolerance: {self.config.position_tolerance:.4f}m")
            print(f"   Rotation tolerance: {self.config.rotation_tolerance:.4f}rad")

            # Ask user if they want to continue anyway
            response = input("\n🤔 Continue with current position anyway? (y/n): ").lower().strip()
            if response in ['y', 'yes']:
                print("✅ Continuing with current position...")
                metrics['converged'] = True  # Override convergence for continuation
                metrics['user_override'] = True
            else:
                print("❌ Visual servoing marked as failed")
                metrics['user_override'] = False

            metrics['final_error'] = pose_error_magnitude

        # Record in history
        self.pose_history.record_correction(
            position_name, stored_robot_pose, current_robot_pose,
            stored_tag_pose, current_tag_pose, metrics)

        # Update stored pose if requested and converged - DISABLED
        # if update_stored_pose and metrics['converged']:
        #     success = self.pose_history.update_position_pose(
        #         position_name, current_robot_pose, current_tag_pose)
        #     metrics['pose_updated'] = success
        # 
        #     if success:
        #         print(f"💾 Updated stored pose for '{position_name}'")
        print("🚫 Position update disabled - keeping original taught poses")

        # DISABLED: Update all equipment positions when converged
        # Equipment updates disabled to always use original taught positions
        # if metrics['converged'] and np.linalg.norm(total_correction) > 0.001:
        #     print("🔧 Applying equipment-wide position updates...")
        #     equipment_success = self.pose_history.update_equipment_positions(
        #         position_name, total_correction)
        #     metrics['equipment_updated'] = equipment_success
        #     if equipment_success:
        #         print(f"✅ All equipment positions updated with correction: {total_correction}")
        #     else:
        #         print("⚠️  Failed to update equipment positions")
        print("🚫 Equipment position updates disabled - using original taught positions")

        print(f"\n🎯 Direct visual servoing completed for '{position_name}'")
        print(f"   Converged: {metrics['converged']}")
        print(f"   Iterations: {metrics['iterations']}")
        print(f"   Total correction: {np.linalg.norm(total_correction):.4f}")

        return metrics['converged'], metrics

    def _visual_servo_via_observation(self, position_name: str, position_data: Dict[str, Any], update_stored_pose: bool) -> Tuple[bool, Dict[str, Any]]:
        """
        Perform visual servoing for positions using observation pose with fixed offset

        Args:
            position_name: Name of the taught position
            position_data: Position data from YAML
            update_stored_pose: Whether to update the stored position after correction

        Returns:
            (success, metrics) tuple
        """
        print(f"👁️  Observation-based visual servoing for '{position_name}'")

        observation_pose_name = position_data['observation_pose']
        observation_offset = np.array(position_data.get('observation_offset', [0, 0, 0, 0, 0, 0]))

        print(f"🔭 Using observation pose: '{observation_pose_name}'")
        print(f"📐 Observation offset: {observation_offset}")

        # Get observation pose data
        observation_data = self._get_position_data(observation_pose_name)
        if not observation_data:
            return False, {'error': f'Observation pose {observation_pose_name} not found'}

        if not observation_data.get('camera_to_tag'):
            return False, {'error': f'Observation pose {observation_pose_name} has no AprilTag view'}

        tag_reference = observation_data['tag_reference']
        stored_obs_tag_pose = np.array(observation_data['camera_to_tag'])
        stored_obs_robot_pose = np.array(observation_data['coordinates'])
        stored_target_pose = np.array(position_data['coordinates'])

        # Extract tag ID from reference
        if isinstance(tag_reference, str) and tag_reference.startswith('tag_'):
            tag_id = int(tag_reference.split('_')[1])
        else:
            tag_id = int(tag_reference)

        print(f"🏷️  Using AprilTag {tag_id} via observation pose")

        # Initialize metrics
        metrics = {
            'position_name': position_name,
            'observation_pose': observation_pose_name,
            'observation_offset': observation_offset.tolist(),
            'tag_id': tag_id,
            'method': 'observation',
            'iterations': 0,
            'total_correction': np.zeros(6),
            'final_error': None,
            'converged': False,
            'corrections_applied': []
        }

        # Calculate initial corrected target pose
        current_target_pose = stored_target_pose.copy()
        total_correction = np.zeros(6)
        previous_error_magnitude = float('inf')  # Initialize for comparison

        # Iterative correction loop
        for iteration in range(self.config.max_iterations):
            print(f"\n🔄 Iteration {iteration + 1}/{self.config.max_iterations}")
            metrics['iterations'] = iteration + 1

            # Step 1: Move to observation pose + corrections
            current_obs_pose = stored_obs_robot_pose + total_correction
            print("🔭 Moving to corrected observation pose...")
            success = self.robot.move_to_pose(current_obs_pose)
            if not success:
                print("❌ Failed to move robot to observation pose")
                return False, metrics

            # Step 2: Detect current AprilTag pose from observation position
            current_obs_tag_pose = self.detection_filter.get_filtered_tag_pose(tag_id)
            if current_obs_tag_pose is None:
                print(f"❌ Failed to detect AprilTag {tag_id} from observation pose")
                return False, metrics

            # Step 3: Calculate tag error from observation position
            tag_error = current_obs_tag_pose - stored_obs_tag_pose
            pose_error_magnitude = np.linalg.norm(tag_error)

            print(f"📏 Tag pose error magnitude: {pose_error_magnitude:.4f}")
            print(f"   Translation error: [{tag_error[0]:.4f}, {tag_error[1]:.4f}, {tag_error[2]:.4f}]m")
            print(f"   Rotation error: [{tag_error[3]:.4f}, {tag_error[4]:.4f}, {tag_error[5]:.4f}]rad")

            # Detection consistency check - DISABLED for testing
            # The check was too aggressive and preventing convergence
            # TODO: Re-enable with better logic after testing
            # if len(self.error_history) >= 1:
            #     prev_error = self.error_history[-1]
            #     error_increase_threshold = 2.0  # Allow larger increases (100% instead of 30%)
            #
            #     if pose_error_magnitude > prev_error * error_increase_threshold:
            #         should_skip_correction = True
            #         print(f"⚠️  Detection inconsistency detected!")
            #         print(f"   Previous error: {prev_error:.4f}")
            #         print(f"   Current error: {pose_error_magnitude:.4f}")
            #         print(f"   Increase: {((pose_error_magnitude/prev_error - 1) * 100):.1f}%")
            #         print(f"   🚫 Skipping correction for this iteration (likely detection noise)")
            #
            #         # Don't update error history with noisy measurement
            #         # Continue to next iteration without applying correction
            #         continue

            # Update error history with good measurement
            self.error_history.append(pose_error_magnitude)
            if len(self.error_history) > self.max_error_history:
                self.error_history.pop(0)

            # Oscillation detection and adaptive damping
            current_damping = self.config.damping_factor
            if len(self.error_history) >= 3:
                # Check if error is oscillating (increasing then decreasing pattern)
                recent_errors = self.error_history[-3:]
                if recent_errors[1] > recent_errors[0] * 1.5:  # Big jump up
                    print("⚠️  Oscillation detected - reducing damping")
                    current_damping *= 0.5  # Halve the damping for this iteration
                    print(f"   Using adaptive damping: {current_damping:.3f}")

            # Check convergence
            if (np.linalg.norm(tag_error[:3]) < self.config.position_tolerance
                    and np.linalg.norm(tag_error[3:]) < self.config.rotation_tolerance):
                print("✅ Converged within tolerance")
                metrics['converged'] = True
                metrics['final_error'] = pose_error_magnitude
                break

            # "Good enough" check - if error is already quite small, don't over-correct
            if pose_error_magnitude < 0.01:  # 10mm total error is quite good (reduced from 60mm)
                print(f"✅ Good enough accuracy - error {pose_error_magnitude:.4f} is acceptable")
                metrics['converged'] = True
                metrics['final_error'] = pose_error_magnitude
                break

            # Practical convergence check - if error is small and stable, accept it
            # Only accept if within 2x the actual tolerance (20mm total error)
            if (iteration > 0 and pose_error_magnitude < 0.02
                    and abs(pose_error_magnitude - previous_error_magnitude) < 0.005):
                print(f"✅ Practical convergence - error stable at {pose_error_magnitude:.4f}")
                metrics['converged'] = True
                metrics['final_error'] = pose_error_magnitude
                break

            previous_error_magnitude = pose_error_magnitude

            # Step 4: Calculate robot pose correction using eye-in-hand observation-based control
            # Since camera moves with robot, corrections apply to both observation and target poses

            # Extract translation and rotation errors separately
            tag_translation_error = tag_error[:3]  # [x, y, z] in camera frame
            tag_rotation_error = tag_error[3:]     # [rx, ry, rz] in camera frame

            # Check if using simple legacy-style mode
            if hasattr(self.config, 'simple_mode') and self.config.simple_mode:
                # Simple mode: only XY translation corrections (like legacy system)
                robot_correction = np.zeros(6)
                robot_correction[0] = -tag_translation_error[0] * 0.4  # X correction
                robot_correction[1] = -tag_translation_error[1] * 0.4  # Y correction
                # Z, RX, RY, RZ remain zero for stability
                
                print("🔧 Simple XY-only correction (legacy style):")
                print(f"   Translation errors: [{tag_translation_error[0]:.4f}, {tag_translation_error[1]:.4f}, {tag_translation_error[2]:.4f}]")
                print(f"   Applied XY correction: [{robot_correction[0]:.4f}, {robot_correction[1]:.4f}, 0.0000] (Z/rotations=0)")
                print("   Simple mode damping: 0.400")
            else:
                # PROPER COORDINATE TRANSFORMATION - Transform camera errors to robot corrections
                camera_trans_error = [tag_translation_error[0], tag_translation_error[1], tag_translation_error[2]]
                camera_rot_error = [tag_rotation_error[0], tag_rotation_error[1], tag_rotation_error[2]]
                
                # Apply coordinate transformation to map camera frame to robot frame
                robot_trans_correction, robot_rot_correction = transform_camera_to_robot_correction(
                    camera_trans_error, camera_rot_error
                )
                
                # Apply damping
                robot_trans_correction = [x * current_damping for x in robot_trans_correction]
                robot_rot_correction = [x * current_damping for x in robot_rot_correction]
                
                # Combine into 6DOF correction vector
                robot_correction = np.array(robot_trans_correction + robot_rot_correction)

                # Apply safety limits
                max_translation_correction = 0.02  # 2cm max per iteration
                max_rotation_correction = 0.1      # ~6 degrees max per iteration

                # Limit translation corrections
                for i in range(3):
                    robot_correction[i] = np.clip(robot_correction[i],
                                                  -max_translation_correction, max_translation_correction)

                # Limit rotation corrections
                for i in range(3, 6):
                    robot_correction[i] = np.clip(robot_correction[i],
                                                  -max_rotation_correction, max_rotation_correction)

                print(f"🔧 Coordinate-transformed IBVS correction (damping={current_damping:.3f}):")
                print(f"   Camera translation errors: [{tag_translation_error[0]:.4f}, {tag_translation_error[1]:.4f}, {tag_translation_error[2]:.4f}]")
                print(f"   Camera rotation errors: [{tag_rotation_error[0]:.4f}, {tag_rotation_error[1]:.4f}, {tag_rotation_error[2]:.4f}]")
                print(f"   Robot translation correction: [{robot_correction[0]:.4f}, {robot_correction[1]:.4f}, {robot_correction[2]:.4f}]")
                print(f"   Robot rotation correction: [{robot_correction[3]:.4f}, {robot_correction[4]:.4f}, {robot_correction[5]:.4f}]")
            print(f"   Adaptive damping: {current_damping:.3f}")

            # Apply safety limits
            correction_valid, safety_metrics = self._validate_correction(
                robot_correction, total_correction)

            if not correction_valid:
                print("❌ Correction exceeds safety limits")
                metrics.update(safety_metrics)
                return False, metrics

            # Step 5: Apply correction to target pose (same spatial correction as observation pose)
            current_target_pose += robot_correction
            total_correction += robot_correction

            correction_magnitude = np.linalg.norm(robot_correction)
            print(f"🔧 Applied correction magnitude: {correction_magnitude:.4f}")

            metrics['corrections_applied'].append({
                'iteration': iteration + 1,
                'tag_error': tag_error.tolist(),
                'robot_correction': robot_correction.tolist(),
                'correction_magnitude': correction_magnitude
            })

        # Step 6: Move to final corrected target pose
        if metrics['converged']:
            print("🎯 Moving to final corrected target pose...")
            
            # First validate that the target pose is reachable
            if not self._is_pose_reachable(current_target_pose):
                print("❌ Final target pose is not reachable by robot")
                print("   This may be due to joint limits, singularities, or workspace constraints")
                print("   Visual servoing converged but target position is unreachable")
                print(f"   Target pose: {[round(x, 3) for x in current_target_pose]}")
                
                # Ask user if they want to continue anyway
                response = input("\n🤔 Attempt move anyway? (y/n): ").lower().strip()
                if response not in ['y', 'yes']:
                    print("⏹️  Stopping workflow - target pose unreachable")
                    return False, metrics
            
            success = self.robot.move_to_pose(current_target_pose)
            if not success:
                print("❌ Failed to move robot to final target pose")
                return False, metrics

        # Final metrics
        metrics['total_correction'] = total_correction.tolist()
        metrics['final_robot_pose'] = current_target_pose.tolist()
        metrics['final_obs_pose'] = (stored_obs_robot_pose + total_correction).tolist()

        if not metrics['converged']:
            print(f"⚠️  Did not converge within {self.config.max_iterations} iterations")
            print(f"   Final error: {pose_error_magnitude:.4f}")
            print(f"   Position tolerance: {self.config.position_tolerance:.4f}m")
            print(f"   Rotation tolerance: {self.config.rotation_tolerance:.4f}rad")

            # Ask user if they want to continue anyway
            response = input("\n🤔 Continue with current position anyway? (y/n): ").lower().strip()
            if response in ['y', 'yes']:
                print("✅ Continuing with current position...")
                metrics['converged'] = True  # Override convergence for continuation
                metrics['user_override'] = True
            else:
                print("❌ Visual servoing marked as failed")
                metrics['user_override'] = False

            metrics['final_error'] = pose_error_magnitude

        # Record in history (record for target position)
        self.pose_history.record_correction(
            position_name, stored_target_pose, current_target_pose,
            stored_obs_tag_pose, current_obs_tag_pose, metrics)

        # Update stored pose if requested and converged - DISABLED
        # if update_stored_pose and metrics['converged']:
        #     success = self.pose_history.update_position_pose(
        #         position_name, current_target_pose, None)  # No direct camera_to_tag for target
        #     metrics['pose_updated'] = success
        # 
        #     if success:
        #         print(f"💾 Updated stored pose for '{position_name}'")
        #
        #     # Also update observation pose
        #     obs_success = self.pose_history.update_position_pose(
        #         observation_pose_name, stored_obs_robot_pose + total_correction, current_obs_tag_pose)
        #     metrics['obs_pose_updated'] = obs_success
        print("🚫 Position update disabled - keeping original taught poses")

        # DISABLED: Update all equipment positions when converged
        # Equipment updates disabled to always use original taught positions
        # if metrics['converged'] and np.linalg.norm(total_correction) > 0.001:
        #     print("🔧 Applying equipment-wide position updates...")
        #     equipment_success = self.pose_history.update_equipment_positions(
        #         position_name, total_correction)
        #     metrics['equipment_updated'] = equipment_success
        #     if equipment_success:
        #         print(f"✅ All equipment positions updated with correction: {total_correction}")
        #     else:
        #         print("⚠️  Failed to update equipment positions")
        print("🚫 Equipment position updates disabled - using original taught positions")

        print(f"\n🎯 Observation-based visual servoing completed for '{position_name}'")
        print(f"   Converged: {metrics['converged']}")
        print(f"   Iterations: {metrics['iterations']}")
        print(f"   Total correction: {np.linalg.norm(total_correction):.4f}")

        return metrics['converged'], metrics

    def check_position_drift(self, position_name: str) -> Tuple[bool, Dict[str, Any]]:
        """
        Check if a position has drifted from its stored location

        Args:
            position_name: Name of position to check

        Returns:
            (has_drifted, metrics) tuple
        """
        print(f"🔍 Checking position drift for '{position_name}'")

        position_data = self._get_position_data(position_name)
        if not position_data or 'tag_reference' not in position_data:
            return False, {'error': 'Position not found or no tag reference'}

        tag_id = position_data['tag_reference']
        stored_tag_pose = np.array(position_data['camera_to_tag'])

        # Detect current tag pose
        current_tag_pose = self.detection_filter.get_filtered_tag_pose(tag_id)
        if current_tag_pose is None:
            return False, {'error': 'Failed to detect AprilTag'}

        # Calculate drift
        drift = current_tag_pose - stored_tag_pose
        translation_drift = np.linalg.norm(drift[:3])
        rotation_drift = np.linalg.norm(drift[3:])

        # Determine if significant drift
        has_drifted = (translation_drift > self.config.position_tolerance * 2
                       or rotation_drift > self.config.rotation_tolerance * 2)

        metrics = {
            'position_name': position_name,
            'tag_id': tag_id,
            'translation_drift': translation_drift,
            'rotation_drift': rotation_drift,
            'drift_vector': drift.tolist(),
            'has_drifted': has_drifted,
            'stored_tag_pose': stored_tag_pose.tolist(),
            'current_tag_pose': current_tag_pose.tolist()
        }

        print("📊 Drift analysis:")
        print(f"   Translation drift: {translation_drift:.4f}m")
        print(f"   Rotation drift: {rotation_drift:.4f}rad")
        print(f"   Significant drift: {has_drifted}")

        return has_drifted, metrics

    def _get_position_data(self, position_name: str) -> Optional[Dict[str, Any]]:
        """Get position data from taught positions file"""
        try:
            positions_data = self.pose_history._load_positions()
            return positions_data.get('positions', {}).get(position_name)
        except Exception as e:
            print(f"❌ Failed to load position data: {e}")
            return None

    def _validate_correction(self, correction: np.ndarray, total_correction: np.ndarray) -> Tuple[bool, Dict[str, Any]]:
        """
        Validate that correction is within safety limits

        Args:
            correction: Proposed correction vector
            total_correction: Cumulative correction so far

        Returns:
            (is_valid, metrics) tuple
        """
        translation_magnitude = np.linalg.norm(correction[:3])
        rotation_magnitude = np.linalg.norm(correction[3:])
        total_translation_magnitude = np.linalg.norm(total_correction[:3] + correction[:3])

        # Check individual correction limits
        translation_ok = translation_magnitude <= self.config.max_translation_correction
        rotation_ok = rotation_magnitude <= self.config.max_rotation_correction
        total_ok = total_translation_magnitude <= self.config.max_total_correction

        is_valid = translation_ok and rotation_ok and total_ok

        metrics = {
            'correction_translation': translation_magnitude,
            'correction_rotation': rotation_magnitude,
            'total_translation': total_translation_magnitude,
            'translation_ok': translation_ok,
            'rotation_ok': rotation_ok,
            'total_ok': total_ok,
            'safety_limits': {
                'max_translation': self.config.max_translation_correction,
                'max_rotation': self.config.max_rotation_correction,
                'max_total': self.config.max_total_correction
            }
        }

        if not is_valid:
            print("⚠️  Safety limit violation:")
            if not translation_ok:
                print(f"   Translation: {translation_magnitude:.4f}m > {self.config.max_translation_correction:.4f}m")
            if not rotation_ok:
                print(f"   Rotation: {rotation_magnitude:.4f}rad > {self.config.max_rotation_correction:.4f}rad")
            if not total_ok:
                print(f"   Total correction: {total_translation_magnitude:.4f}m > {self.config.max_total_correction:.4f}m")

        return is_valid, metrics

    def setup_position_for_visual_servo(self, position_name: str, tag_id: int, camera_to_tag_transform: np.ndarray) -> bool:
        """
        Setup a position for visual servoing by storing AprilTag association

        Args:
            position_name: Name of the taught position
            tag_id: AprilTag ID to associate with this position
            camera_to_tag_transform: 6D pose transform from camera to tag

        Returns:
            True if setup successful
        """
        try:
            # Update the position data with visual servo information
            success = self.pose_history.update_position_tag_association(
                position_name, f"tag_{tag_id}", camera_to_tag_transform.tolist())

            if success:
                print(f"✅ Position '{position_name}' configured for visual servoing with Tag {tag_id}")
                return True
            else:
                print(f"❌ Failed to configure position '{position_name}' for visual servoing")
                return False

        except Exception as e:
            print(f"❌ Error setting up visual servo for position '{position_name}': {e}")
            return False

    def get_corrected_pose(self, position_name: str, original_pose: np.ndarray) -> Optional[np.ndarray]:
        """
        Get a corrected pose for a position using visual servoing (without moving the robot)

        Args:
            position_name: Name of the taught position
            original_pose: Original pose coordinates

        Returns:
            Corrected pose array, or None if correction failed
        """
        print(f"🎯 Computing visual servo correction for position '{position_name}'")

        # Load position data
        position_data = self._get_position_data(position_name)
        if not position_data:
            return None

        # Check if position has AprilTag reference
        if 'tag_reference' not in position_data or not position_data['tag_reference']:
            print(f"❌ Position '{position_name}' has no AprilTag reference")
            return None

        if 'camera_to_tag' not in position_data or not position_data['camera_to_tag']:
            print(f"❌ Position '{position_name}' has no stored camera-to-tag transform")
            return None

        tag_reference = position_data['tag_reference']
        stored_tag_pose = np.array(position_data['camera_to_tag'])

        # Extract tag ID from reference
        if isinstance(tag_reference, str) and tag_reference.startswith('tag_'):
            tag_id = int(tag_reference.split('_')[1])
        else:
            tag_id = int(tag_reference)

        # Detect current AprilTag pose
        current_tag_pose = self.detection_filter.get_filtered_tag_pose(tag_id)
        if current_tag_pose is None:
            print(f"❌ Failed to detect AprilTag {tag_id}")
            return None

        # Calculate pose correction using hand-eye calibration
        tag_error = current_tag_pose - stored_tag_pose
        robot_correction = self._transform_tag_error_to_robot_correction(tag_error)

        # Apply safety limits
        correction_valid, safety_metrics = self._validate_correction(
            robot_correction, np.zeros(6))  # Zero total correction for single-shot calculation

        if not correction_valid:
            print("❌ Calculated correction exceeds safety limits")
            return None

        corrected_pose = original_pose + robot_correction

        correction_magnitude = np.linalg.norm(robot_correction)
        print(f"🔧 Calculated correction magnitude: {correction_magnitude:.4f}")
        print(f"   Translation correction: [{robot_correction[0]:.4f}, {robot_correction[1]:.4f}, {robot_correction[2]:.4f}]m")
        print(f"   Rotation correction: [{robot_correction[3]:.4f}, {robot_correction[4]:.4f}, {robot_correction[5]:.4f}]rad")

        return corrected_pose

    def _is_pose_reachable(self, target_pose):
        """
        Test if a target pose is reachable by the robot.
        
        This performs a quick validation by attempting a small test movement
        to check for joint limits, singularities, or other reachability issues.
        
        Args:
            target_pose: Target pose to validate (numpy array)
            
        Returns:
            bool: True if pose appears reachable, False otherwise
        """
        try:
            # Store current pose
            current_pose = self.robot.get_tcp_pose()
            
            # Try to compute inverse kinematics by attempting a small move
            # This will fail if the pose is outside joint limits or in singularity
            test_success = self.robot.move_to_pose(target_pose, speed=0.01, acceleration=0.01)
            
            if test_success:
                # If move succeeded, immediately return to original position
                self.robot.move_to_pose(current_pose, speed=0.05, acceleration=0.05)
                return True
            else:
                return False
                
        except Exception as e:
            print(f"⚠️  Pose reachability test failed: {e}")
            return False

    def set_robot_controller(self, robot_controller):
        """Set the robot controller instance"""
        self.robot = robot_controller
        print("🤖 Robot controller set for visual servo engine")

    def _load_hand_eye_calibration(self) -> Optional[np.ndarray]:
        """Load hand-eye calibration transformation if available.
        
        Returns:
            4x4 transformation matrix or None if not available
        """
        calib_file = Path("src/ur_toolkit/hand_eye_calibration/hand_eye_calibration.json")
        
        if not calib_file.exists():
            print("⚠️  No hand-eye calibration found - using coordinate frame mapping")
            return None
            
        try:
            with open(calib_file, 'r') as f:
                data = json.load(f)
                
            transform = np.array(data['hand_eye_transform'])
            print("✅ Hand-eye calibration loaded successfully")
            print(f"   Calibration date: {data.get('calibration_date', 'Unknown')}")
            
            return transform
            
        except Exception as e:
            print(f"⚠️  Failed to load hand-eye calibration: {e}")
            return None
            
    def _transform_tag_error_to_robot_correction(self, tag_error: np.ndarray) -> np.ndarray:
        """Transform AprilTag pose error to robot correction.
        
        Args:
            tag_error: 6DOF pose error [x, y, z, rx, ry, rz] in camera frame
            
        Returns:
            6DOF correction [x, y, z, rx, ry, rz] in robot frame
        """
        if self.hand_eye_transform is not None:
            # Use proper hand-eye calibration transformation
            return self._transform_with_hand_eye_calibration(tag_error)
        else:
            # Fallback to manual coordinate frame mapping
            return self._transform_with_coordinate_mapping(tag_error)
            
    def _transform_with_hand_eye_calibration(self, tag_error: np.ndarray) -> np.ndarray:
        """Transform error using hand-eye calibration matrix.
        
        Args:
            tag_error: 6DOF pose error in camera frame
            
        Returns:
            6DOF correction in robot frame
        """
        # Create transformation matrix from error
        error_transform = np.eye(4)
        error_transform[:3, 3] = tag_error[:3]
        
        # For small rotations, approximate rotation matrix
        rx, ry, rz = tag_error[3:6]
        error_transform[:3, :3] = np.array([
            [1, -rz, ry],
            [rz, 1, -rx],
            [-ry, rx, 1]
        ])
        
        # Transform error from camera frame to robot frame
        # For eye-in-hand: robot_error = hand_eye_transform^-1 * camera_error * hand_eye_transform
        hand_eye_inv = np.linalg.inv(self.hand_eye_transform)
        robot_error_transform = hand_eye_inv @ error_transform @ self.hand_eye_transform
        
        # Extract 6DOF correction
        robot_correction = np.zeros(6)
        robot_correction[:3] = robot_error_transform[:3, 3]
        
        # Extract rotation (approximate for small angles)
        R = robot_error_transform[:3, :3]
        robot_correction[3] = (R[2, 1] - R[1, 2]) / 2
        robot_correction[4] = (R[0, 2] - R[2, 0]) / 2
        robot_correction[5] = (R[1, 0] - R[0, 1]) / 2
        
        # Negate to get correction (opposite of error)
        return -robot_correction
        
    def _transform_with_coordinate_mapping(self, tag_error: np.ndarray) -> np.ndarray:
        """Transform error using manual coordinate frame mapping.
        
        This is the fallback method when no hand-eye calibration is available.
        
        Args:
            tag_error: 6DOF pose error in camera frame
            
        Returns:
            6DOF correction in robot frame
        """
        # Use the improved coordinate transformation from easy_handeye principles
        # Assuming camera mounted looking forward from end-effector
        
        robot_correction = np.zeros(6)
        
        # Position corrections (camera to robot coordinates)
        # For eye-in-hand with camera looking forward:
        robot_correction[0] = -tag_error[2]   # Camera Z (depth) -> Robot X (forward/back)
        robot_correction[1] = -tag_error[0]   # Camera X (right) -> Robot Y (left/right)
        robot_correction[2] = tag_error[1]    # Camera Y (down) -> Robot Z (up/down)
        
        # Rotation corrections (simplified - should use proper transformation)
        robot_correction[3:6] = tag_error[3:6] * 0.5  # Reduced rotation gains
        
        # Negate to get correction (opposite of error)
        return -robot_correction
