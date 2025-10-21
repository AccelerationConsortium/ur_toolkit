#!/usr/bin/env python3
"""
Pose Correction Engine
Robust visual servoing using filtered pose estimates and stable correction algorithms
"""

import cv2
import numpy as np
import time
import json
from typing import Optional, Tuple, Dict, Any
from pathlib import Path

# Import existing hardware interfaces
from ur_toolkit.robots.ur.ur_controller import URController
from ur_toolkit.camera.picam.picam import PiCam, PiCamConfig
from ur_toolkit.apriltag_detection import AprilTagDetector
from ur_toolkit.config_manager import config

# Import new robust components
from .robust_estimator import RobustPoseEstimator
from .kalman_filter import PoseKalmanFilter


class PoseCorrectionEngine:
    """Robust pose correction engine for visual servoing"""
    
    def __init__(self, robot_controller: URController, camera: Optional[PiCam] = None, 
                 apriltag_detector: Optional[AprilTagDetector] = None):
        """
        Initialize pose correction engine
        
        Args:
            robot_controller: Robot controller instance
            camera: Camera instance (will create if None)
            apriltag_detector: AprilTag detector (will create if None)
        """
        self.robot = robot_controller
        
        # Initialize camera if not provided
        if camera is None:
            from ur_toolkit.config_manager import get_camera_host, get_camera_port
            host = get_camera_host()
            port = get_camera_port()
            camera_config = PiCamConfig(hostname=host, port=port)
            self.camera = PiCam(camera_config)
        else:
            self.camera = camera
            
        # Initialize AprilTag detector if not provided
        if apriltag_detector is None:
            self.detector = AprilTagDetector()
        else:
            self.detector = apriltag_detector
            
        # Check if pose estimation is available
        if not self.detector.pose_estimation_enabled:
            raise ValueError("Camera calibration required for pose correction")
            
        # Initialize robust pose estimator
        self.pose_estimator = RobustPoseEstimator(
            camera_matrix=self.detector.camera_matrix,
            dist_coeffs=self.detector.dist_coeffs,
            tag_size=self.detector.tag_size
        )
        
        # Initialize Kalman filter for pose smoothing
        self.kalman_filter = PoseKalmanFilter(
            dt=0.1,
            process_noise=0.01,
            measurement_noise=0.1
        )
        
        # Load hand-eye calibration for eye-in-hand setup
        self.hand_eye_transform = self._load_hand_eye_calibration()
        
        # Configuration - more aggressive for better corrections
        self.max_iterations = config.get('pose_correction.max_iterations', 10)
        self.position_tolerance = config.get('pose_correction.position_tolerance', 0.001)  # 1mm (tighter)
        self.rotation_tolerance = config.get('pose_correction.rotation_tolerance', 0.017)  # 1 degree (tighter)
        self.max_correction_per_step = config.get('pose_correction.max_correction_per_step', 0.05)  # 5cm (larger)
        self.step_size = config.get('pose_correction.step_size', 1.0)  # 100% of calculated correction (more aggressive)
        
        # State tracking
        self.last_update_time = None
        self.target_pose = None
        
        print("🎯 Pose Correction Engine initialized")
        print(f"   Max iterations: {self.max_iterations}")
        print(f"   Position tolerance: {self.position_tolerance*1000:.1f}mm")
        print(f"   Rotation tolerance: {self.rotation_tolerance*180/np.pi:.1f}°")
    
    def set_target_from_current_detection(self, tag_id: int = 0) -> bool:
        """
        Set target pose from current AprilTag detection
        
        Args:
            tag_id: ID of AprilTag to use as target
            
        Returns:
            True if target was set successfully
        """
        try:
            # Capture image
            print(f"📸 Capturing image to set target for tag {tag_id}...")
            image_path = self.camera.capture_photo()
            if not image_path:
                print("❌ Failed to capture image")
                return False
                
            # Detect tags
            image = cv2.imread(image_path)
            detections = self.detector.detect_tags(image)
            
            # Find target tag
            target_detection = None
            for detection in detections:
                if detection['tag_id'] == tag_id:
                    target_detection = detection
                    break
                    
            if target_detection is None:
                print(f"❌ Tag {tag_id} not found in image")
                return False
                
            # Get robust pose estimate
            pose_result = self.pose_estimator.estimate_pose_ransac(target_detection)
            if pose_result is None:
                print("❌ Failed to estimate target pose")
                return False
                
            rvec, tvec, num_inliers = pose_result
            
            # Validate pose
            if not self.pose_estimator.validate_pose(rvec, tvec):
                print("❌ Target pose validation failed")
                return False
                
            # Convert to 6DOF and store
            raw_target_pose = self.pose_estimator.pose_to_6dof(rvec, tvec)
            
            # Apply coordinate frame correction to target pose as well
            self.target_pose = self._apply_coordinate_frame_correction(raw_target_pose)
            
            print(f"✅ Target pose set from tag {tag_id} ({num_inliers}/4 inliers)")
            print(f"   Position: [{self.target_pose[0]:.3f}, {self.target_pose[1]:.3f}, {self.target_pose[2]:.3f}]m")
            target_rot_deg = self._rad_to_deg(self.target_pose[3:])
            print(f"   Rotation: [{target_rot_deg[0]:.1f}, {target_rot_deg[1]:.1f}, {target_rot_deg[2]:.1f}]deg")
            
            # Diagnostic: Check for potential frame mismatch
            if np.any(np.abs(self.target_pose[3:]) > 3.0):  # > ~172 degrees
                print("⚠️  WARNING: Large rotation detected in target pose!")
                print("   This might indicate a coordinate frame mismatch between camera and expected orientation")
            
            return True
            
        except Exception as e:
            print(f"❌ Error setting target pose: {e}")
            return False
    
    def set_target_from_taught_position(self, position_name: str) -> bool:
        """
        Set target pose from taught position's stored camera-to-tag relationship
        
        Args:
            position_name: Name of taught position containing camera_to_tag data
            
        Returns:
            True if target was set successfully
        """
        try:
            # Load taught positions
            from pathlib import Path
            import yaml
            
            positions_file = Path(__file__).parent.parent / "positions" / "taught_positions.yaml"
            with open(positions_file, 'r') as f:
                taught_positions = yaml.safe_load(f)
            
            # Find the position
            positions = taught_positions.get('positions', {})
            if position_name not in positions:
                print(f"❌ Position '{position_name}' not found in taught positions")
                return False
                
            position_data = positions[position_name]
            camera_to_tag = position_data.get('camera_to_tag')
            
            if camera_to_tag is None:
                print(f"❌ Position '{position_name}' has no camera_to_tag data")
                return False
                
            if len(camera_to_tag) != 6:
                print(f"❌ Invalid camera_to_tag data for '{position_name}': {camera_to_tag}")
                return False
                
            # Convert to numpy array and apply coordinate frame correction
            raw_target_pose = np.array(camera_to_tag, dtype=float)
            self.target_pose = self._apply_coordinate_frame_correction(raw_target_pose)
            
            print(f"✅ Target pose set from taught position '{position_name}'")
            print(f"   Original taught relationship: {raw_target_pose}")
            print(f"   Position: [{self.target_pose[0]:.3f}, {self.target_pose[1]:.3f}, {self.target_pose[2]:.3f}]m")
            target_rot_deg = self._rad_to_deg(self.target_pose[3:])
            print(f"   Rotation: [{target_rot_deg[0]:.1f}, {target_rot_deg[1]:.1f}, {target_rot_deg[2]:.1f}]deg")
            
            return True
            
        except Exception as e:
            print(f"❌ Error setting target from taught position: {e}")
            return False

    def correct_to_target(self, tag_id: int = 0) -> Tuple[bool, Dict[str, Any]]:
        """
        Correct robot pose to match target using robust visual servoing
        
        Args:
            tag_id: ID of AprilTag to track
            
        Returns:
            Tuple of (success, metrics)
        """
        if self.target_pose is None:
            print("❌ No target pose set. Call set_target_from_current_detection() first")
            return False, {'error': 'no_target'}
            
        print(f"🎯 Starting pose correction to target (tag {tag_id})")
        
        metrics = {
            'iterations': 0,
            'converged': False,
            'final_error': None,
            'corrections_applied': [],
            'pose_history': []
        }
        
        # Reset Kalman filter for new correction sequence
        self.kalman_filter.reset()
        
        try:
            for iteration in range(self.max_iterations):
                print(f"\n🔄 Iteration {iteration + 1}/{self.max_iterations}")
                metrics['iterations'] = iteration + 1
                
                # Get current time for dt calculation
                current_time = time.time()
                dt = 0.1 if self.last_update_time is None else current_time - self.last_update_time
                self.last_update_time = current_time
                
                # Capture and analyze current state
                current_pose = self._get_current_filtered_pose(tag_id, dt)
                if current_pose is None:
                    print("❌ Failed to get current pose")
                    continue
                    
                metrics['pose_history'].append(current_pose.tolist())
                
                # Calculate error (eye-in-hand: error is how much AprilTag has moved in camera view)
                pose_error = self.target_pose - current_pose  # Target minus current
                error_magnitude = np.linalg.norm(pose_error)
                
                print(f"📏 Pose error magnitude: {error_magnitude:.4f}")
                print(f"   Translation error: [{pose_error[0]:.4f}, {pose_error[1]:.4f}, {pose_error[2]:.4f}]m")
                rotation_error_deg = self._rad_to_deg(pose_error[3:])
                print(f"   Rotation error: [{rotation_error_deg[0]:.1f}, {rotation_error_deg[1]:.1f}, {rotation_error_deg[2]:.1f}]deg")
                print(f"   Rotation error (rad): [{pose_error[3]:.4f}, {pose_error[4]:.4f}, {pose_error[5]:.4f}]rad")
                
                # Check convergence
                if (np.linalg.norm(pose_error[:3]) < self.position_tolerance and 
                    np.linalg.norm(pose_error[3:]) < self.rotation_tolerance):
                    print("✅ Converged to target pose")
                    metrics['converged'] = True
                    metrics['final_error'] = error_magnitude
                    break
                
                # Calculate and apply correction
                correction = self._calculate_stable_correction(pose_error)
                if correction is None:
                    print("❌ Failed to calculate correction")
                    continue
                    
                # Apply correction to robot
                if not self._apply_correction(correction, error_magnitude):
                    print("❌ Failed to apply correction")
                    continue
                    
                metrics['corrections_applied'].append(correction.tolist())
                
                # Brief pause for system to settle
                time.sleep(0.2)
                
            if not metrics['converged']:
                final_pose = self._get_current_filtered_pose(tag_id, 0.1)
                if final_pose is not None:
                    final_error = np.linalg.norm(self.target_pose - final_pose)
                    metrics['final_error'] = final_error
                    print(f"⚠️  Did not converge within {self.max_iterations} iterations")
                    print(f"   Final error: {final_error:.4f}")
                    
            return metrics['converged'], metrics
            
        except Exception as e:
            print(f"❌ Pose correction error: {e}")
            metrics['error'] = str(e)
            return False, metrics
    
    def _get_current_filtered_pose(self, tag_id: int, dt: float) -> Optional[np.ndarray]:
        """
        Get current filtered pose estimate
        
        Args:
            tag_id: AprilTag ID to detect
            dt: Time step for filtering
            
        Returns:
            Filtered 6DOF pose or None if detection failed
        """
        try:
            # Capture image
            image_path = self.camera.capture_photo()
            if not image_path:
                return None
                
            # Detect tags
            image = cv2.imread(image_path)
            detections = self.detector.detect_tags(image)
            
            # Find target tag
            target_detection = None
            for detection in detections:
                if detection['tag_id'] == tag_id:
                    target_detection = detection
                    break
                    
            if target_detection is None:
                print(f"⚠️  Tag {tag_id} not detected")
                return None
                
            # Get robust pose estimate
            pose_result = self.pose_estimator.estimate_pose_ransac(target_detection)
            if pose_result is None:
                return None
                
            rvec, tvec, num_inliers = pose_result
            
            # Validate pose
            if not self.pose_estimator.validate_pose(rvec, tvec):
                return None
                
            # Convert to 6DOF
            raw_pose = self.pose_estimator.pose_to_6dof(rvec, tvec)
            
            # Apply coordinate frame correction to handle 180° orientation mismatch
            corrected_pose = self._apply_coordinate_frame_correction(raw_pose)
            
            # Apply Kalman filtering
            filtered_pose = self.kalman_filter.update(corrected_pose, dt)
            
            return filtered_pose
            
        except Exception as e:
            print(f"❌ Error getting current pose: {e}")
            return None
    
    def _calculate_stable_correction(self, pose_error: np.ndarray) -> Optional[np.ndarray]:
        """
        Calculate stable correction using pose transformation approach
        
        Args:
            pose_error: 6DOF pose error [x, y, z, rx, ry, rz]
            
        Returns:
            6DOF correction vector or None if calculation failed
        """
        try:
            # Apply step size to prevent overshooting
            correction = pose_error * self.step_size
            
            # Apply safety limits
            translation_correction = correction[:3]
            rotation_correction = correction[3:]
            
            # Limit translation magnitude
            translation_magnitude = np.linalg.norm(translation_correction)
            if translation_magnitude > self.max_correction_per_step:
                translation_correction = (translation_correction / translation_magnitude) * self.max_correction_per_step
                
            # Limit rotation magnitude (max 10 degrees per step)
            max_rotation_per_step = 0.175  # ~10 degrees
            rotation_magnitude = np.linalg.norm(rotation_correction)
            if rotation_magnitude > max_rotation_per_step:
                rotation_correction = (rotation_correction / rotation_magnitude) * max_rotation_per_step
                
            final_correction = np.concatenate([translation_correction, rotation_correction])
            
            print(f"🔧 Calculated correction (step_size={self.step_size})")
            print(f"   Translation: [{final_correction[0]:.4f}, {final_correction[1]:.4f}, {final_correction[2]:.4f}]m")
            rotation_correction_deg = self._rad_to_deg(final_correction[3:])
            print(f"   Rotation: [{rotation_correction_deg[0]:.1f}, {rotation_correction_deg[1]:.1f}, {rotation_correction_deg[2]:.1f}]deg")
            
            return final_correction
            
        except Exception as e:
            print(f"❌ Error calculating correction: {e}")
            return None
    
    def _apply_correction(self, correction: np.ndarray, error_magnitude: float) -> bool:
        """
        Apply correction to robot pose with proper coordinate transformation
        
        Args:
            correction: 6DOF correction vector in camera frame
            
        Returns:
            True if correction applied successfully
        """
        try:
            # Get current robot pose
            current_robot_pose = self.robot.get_tcp_pose()
            
            # Transform camera-frame correction to robot frame using hand-eye calibration
            camera_rot_deg = self._rad_to_deg(correction[3:])
            print(f"🔧 Camera correction: [{correction[0]:.4f}, {correction[1]:.4f}, {correction[2]:.4f}]m, [{camera_rot_deg[0]:.1f}, {camera_rot_deg[1]:.1f}, {camera_rot_deg[2]:.1f}]deg")
            
            if self.hand_eye_transform is not None:
                # Use much more aggressive scaling for meaningful corrections
                if error_magnitude < 0.01:  # Very close to target, be conservative
                    robot_correction = -correction * 0.5  # 50%
                elif error_magnitude < 0.05:  # Close to target, moderate scaling
                    robot_correction = -correction * 0.8  # 80%
                else:  # Far from target, full corrections  
                    robot_correction = -correction * 1.0  # 100%
                print(f"✅ Using aggressive scaling (hand-eye calibration available but bypassed)")
            else:
                # Fallback: full scaling (eye-in-hand requires inverse)
                robot_correction = -correction * 1.0  # Negative for eye-in-hand, full correction
                print(f"⚠️  No hand-eye calibration, using full inverse scaling")
                
            robot_rot_deg = self._rad_to_deg(robot_correction[3:])
            print(f"🔧 Robot correction: [{robot_correction[0]:.4f}, {robot_correction[1]:.4f}, {robot_correction[2]:.4f}]m, [{robot_rot_deg[0]:.1f}, {robot_rot_deg[1]:.1f}, {robot_rot_deg[2]:.1f}]deg")
            
            # Apply correction with proper pose composition
            target_robot_pose = self._compose_pose_correction(current_robot_pose, robot_correction)
            
            # Move robot to corrected pose
            success = self.robot.move_to_pose(target_robot_pose, speed=0.02, wait=True)
            
            if success:
                print(f"✅ Applied correction to robot")
            else:
                print(f"❌ Failed to move robot to corrected pose")
                
            return success
            
        except Exception as e:
            print(f"❌ Error applying correction: {e}")
            return False
    
    def _transform_camera_to_robot_correction(self, camera_correction: np.ndarray) -> np.ndarray:
        """
        Transform camera frame correction to robot frame
        
        Args:
            camera_correction: 6DOF correction in camera frame
            
        Returns:
            6DOF correction in robot frame
        """
        # For this simplified version, we'll assume the camera is mounted looking down
        # with Z-axis pointing toward the robot's work surface
        # This transform should be replaced with proper hand-eye calibration matrix
        
        # Simple approximate transformation (camera looking down at workspace)
        robot_correction = camera_correction.copy()
        
        # Flip Y and Z to account for camera orientation
        # Camera: +X=right, +Y=down, +Z=forward (into scene)
        # Robot: +X=forward, +Y=left, +Z=up
        robot_correction[0] = -camera_correction[2]  # Camera Z → Robot -X (back/forward)
        robot_correction[1] = -camera_correction[0]  # Camera X → Robot -Y (left/right) 
        robot_correction[2] = camera_correction[1]   # Camera Y → Robot Z (up/down)
        
        # Scale down the correction to be more conservative
        robot_correction *= 0.3  # Reduce correction magnitude
        
        # Limit rotation corrections which are most problematic
        robot_correction[3:] *= 0.1  # Very conservative rotation corrections
        
        return robot_correction
    
    def _compose_pose_correction(self, current_pose: np.ndarray, correction: np.ndarray) -> np.ndarray:
        """
        Properly compose pose with correction (not simple addition)
        
        Args:
            current_pose: Current robot pose [x,y,z,rx,ry,rz]
            correction: Pose correction [dx,dy,dz,drx,dry,drz]
            
        Returns:
            Target pose with correction applied
        """
        # For small corrections, simple addition is approximately correct
        # For larger corrections, proper pose composition should be used
        target_pose = current_pose + correction
        
        # Ensure rotation angles stay within reasonable bounds
        target_pose[3:] = self._normalize_angles(target_pose[3:])
        
        return target_pose
    
    def _normalize_angles(self, angles: np.ndarray) -> np.ndarray:
        """Normalize angles to [-π, π] range"""
        normalized = angles.copy()
        for i in range(len(normalized)):
            while normalized[i] > np.pi:
                normalized[i] -= 2 * np.pi
            while normalized[i] < -np.pi:
                normalized[i] += 2 * np.pi
        return normalized
    
    def _rad_to_deg(self, rad_array: np.ndarray) -> np.ndarray:
        """Convert radians to degrees for readable logging"""
        return np.rad2deg(rad_array)
    
    def _apply_coordinate_frame_correction(self, pose_6dof: np.ndarray) -> np.ndarray:
        """
        Apply coordinate frame correction to handle 180° orientation mismatch
        
        The AprilTag detection shows ~180° rotation in X and Z axes compared to expected
        coordinate frame. This method corrects for this systematic offset.
        
        Args:
            pose_6dof: Raw 6DOF pose [x, y, z, rx, ry, rz] 
            
        Returns:
            Corrected 6DOF pose with coordinate frame alignment
        """
        corrected_pose = pose_6dof.copy()
        
        # Apply 180° rotation correction around Y-axis to align coordinate frames
        # This compensates for the systematic orientation offset discovered in diagnostics
        corrected_pose[3] += np.pi  # Add 180° to X-rotation (around Y-axis)
        corrected_pose[5] += np.pi  # Add 180° to Z-rotation (around Y-axis)
        
        # Normalize angles to [-π, π] range
        corrected_pose[3] = np.arctan2(np.sin(corrected_pose[3]), np.cos(corrected_pose[3]))
        corrected_pose[5] = np.arctan2(np.sin(corrected_pose[5]), np.cos(corrected_pose[5]))
        
        return corrected_pose
    
    def _load_hand_eye_calibration(self) -> Optional[np.ndarray]:
        """
        Load hand-eye calibration transformation matrix
        
        Returns:
            4x4 transformation matrix from camera to end-effector frame
        """
        try:
            calib_file = Path("src/ur_toolkit/hand_eye_calibration/hand_eye_calibration.json")
            if not calib_file.exists():
                print("⚠️  Hand-eye calibration file not found, using simple transformation")
                return None
                
            with open(calib_file, 'r') as f:
                data = json.load(f)
                
            # Extract transformation matrix
            transform = np.array(data['hand_eye_transform']['matrix'])
            print(f"✅ Loaded hand-eye calibration from: {calib_file}")
            return transform
            
        except Exception as e:
            print(f"❌ Error loading hand-eye calibration: {e}")
            return None
    
    def _transform_with_hand_eye_calibration(self, camera_correction: np.ndarray, error_magnitude: float) -> np.ndarray:
        """
        Transform camera frame correction to robot frame using hand-eye calibration
        
        Args:
            camera_correction: 6DOF correction in camera frame [x,y,z,rx,ry,rz]
            
        Returns:
            6DOF correction in robot frame
        """
        try:
            # Convert 6DOF correction to homogeneous transformation matrix
            translation = camera_correction[:3]
            rotation = camera_correction[3:]
            
            # Create rotation matrix from rotation vector
            rotation_matrix, _ = cv2.Rodrigues(rotation)
            
            # Create 4x4 transformation matrix
            camera_transform = np.eye(4)
            camera_transform[:3, :3] = rotation_matrix
            camera_transform[:3, 3] = translation
            
            # For eye-in-hand: robot_correction = inv(hand_eye) @ camera_correction @ hand_eye
            # But for small corrections, we can use the simpler linear transformation
            hand_eye_inv = np.linalg.inv(self.hand_eye_transform)
            
            # Transform the correction
            robot_transform = hand_eye_inv @ camera_transform @ self.hand_eye_transform
            
            # Extract 6DOF from transformation matrix
            robot_translation = robot_transform[:3, 3]
            robot_rotation_matrix = robot_transform[:3, :3]
            robot_rotation, _ = cv2.Rodrigues(robot_rotation_matrix)
            
            # For eye-in-hand, we typically need to invert the correction direction
            # because moving the robot moves the camera in the opposite direction
            # Use adaptive scaling based on error magnitude
            if error_magnitude < 0.2:  # Close to target, be more conservative
                translation_scale = 0.01
                rotation_scale = 0.005
            else:  # Far from target, normal corrections
                translation_scale = 0.03
                rotation_scale = 0.01
                
            robot_correction = np.concatenate([-robot_translation * translation_scale, -robot_rotation.flatten() * rotation_scale])
            
            return robot_correction
            
        except Exception as e:
            print(f"❌ Error in hand-eye transformation: {e}")
            # Fallback to simple inverse scaling
            return -camera_correction * 0.05