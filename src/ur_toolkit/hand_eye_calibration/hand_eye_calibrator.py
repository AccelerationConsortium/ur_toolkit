"""
Hand-Eye Calibration Module for UR Toolkit
Adapted from Zivid's proven hand-eye calibration approach.

This module provides automated hand-eye calibration using AprilTags
and multiple robot poses to solve the AX = XB calibration problem.
"""

import datetime
import json
import time
from pathlib import Path
from typing import Dict, List, Tuple, Optional
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R

from ..robots.ur.ur_controller import URController
from ..apriltag_detection import AprilTagDetector
from ..camera.picam.picam import PiCam


class HandEyeCalibrator:
    """Hand-eye calibration using AprilTags and UR robot."""

    def __init__(
        self,
        robot_controller: URController,
        camera: PiCam,
        apriltag_detector: AprilTagDetector,
        tag_ids: List[int] = [0]
    ):
        """Initialize hand-eye calibrator.

        Args:
            robot_controller: UR robot controller
            camera: Camera interface
            apriltag_detector: AprilTag detector
            tag_ids: List of AprilTag IDs to use for calibration
        """
        self.robot = robot_controller
        self.camera = camera
        self.detector = apriltag_detector
        self.tag_ids = tag_ids

        # Calibration data storage
        self.robot_poses: List[np.ndarray] = []
        self.tag_poses: List[np.ndarray] = []
        self.calibration_images: List[np.ndarray] = []

        # Results
        self.hand_eye_transform: Optional[np.ndarray] = None
        self.calibration_residuals: List[Dict] = []

    def generate_calibration_poses(self, num_poses: int = 15) -> List[np.ndarray]:
        """Generate diverse robot poses for calibration.

        Following Zivid's recommendation of 10-20 poses with good diversity.

        Args:
            num_poses: Number of calibration poses to generate

        Returns:
            List of 4x4 transformation matrices representing robot poses
        """
        # Get current robot pose as reference (as 4x4 matrix)
        current_pose = self.robot.get_pose_matrix()

        poses = []

        # Generate poses with varied orientations and positions
        # This ensures good observability for hand-eye calibration
        for i in range(num_poses):
            # Vary position around current location (±100mm)
            pos_offset = np.array([
                np.random.uniform(-0.1, 0.1),  # ±100mm in X
                np.random.uniform(-0.1, 0.1),  # ±100mm in Y
                np.random.uniform(-0.05, 0.05)  # ±50mm in Z
            ])

            # Vary orientation (±30 degrees)
            rot_offset = np.array([
                np.random.uniform(-np.pi / 6, np.pi / 6),  # ±30° around X
                np.random.uniform(-np.pi / 6, np.pi / 6),  # ±30° around Y
                np.random.uniform(-np.pi / 4, np.pi / 4)   # ±45° around Z
            ])

            # Create transformation matrix
            pose = np.eye(4)
            pose[:3, 3] = current_pose[:3, 3] + pos_offset

            # Combine current rotation with offset
            current_rot = R.from_matrix(current_pose[:3, :3])
            offset_rot = R.from_rotvec(rot_offset)
            new_rot = current_rot * offset_rot
            pose[:3, :3] = new_rot.as_matrix()

            poses.append(pose)

        return poses

    def move_to_pose_safely(self, target_pose: np.ndarray) -> bool:
        """Move robot to calibration pose with safety checks.

        Args:
            target_pose: 4x4 transformation matrix

        Returns:
            True if move successful, False otherwise
        """
        try:
            # Move to pose (convert from matrix to 1D format)
            target_pose_1d = self.robot.matrix_to_pose(target_pose)
            success = self.robot.move_to_pose(target_pose_1d, speed=0.1)
            if not success:
                print("⚠️  Failed to move to pose")
                return False

            # Wait for robot to settle
            time.sleep(1.0)

            return True

        except Exception as e:
            print(f"❌ Error moving to pose: {e}")
            return False

    def capture_calibration_data(self, pose_index: int) -> Optional[Tuple[np.ndarray, Dict]]:
        """Capture image and detect AprilTag at current robot pose.

        Args:
            pose_index: Index of current calibration pose

        Returns:
            Tuple of (robot_pose, tag_detection_results) or None if failed
        """
        try:
            # Capture image
            image_path = self.camera.capture_photo()
            if image_path is None:
                print("❌ Failed to capture image")
                return None

            # Load image for processing
            import cv2
            image = cv2.imread(image_path)
            if image is None:
                print("❌ Failed to load captured image")
                return None

            self.calibration_images.append(image.copy())

            # Detect AprilTags
            detections = self.detector.detect_tags(image)

            # Check if we found the calibration tags
            valid_detections = {}
            for detection in detections:
                tag_id = detection['tag_id']
                if tag_id in self.tag_ids:
                    # Use decision_margin as quality metric (higher is better)
                    if detection['decision_margin'] > 10.0:  # Good quality threshold
                        valid_detections[tag_id] = detection
                        print(f"✅ Found tag {tag_id} with quality {detection['decision_margin']:.2f}")
                    else:
                        print(f"⚠️  Low quality detection for tag {tag_id}: {detection['decision_margin']:.2f}")

            if not valid_detections:
                print("❌ No valid AprilTag detections found")
                print(f"   Expected tag IDs: {self.tag_ids}")
                if detections:
                    print(f"   Detected tags: {[d['tag_id'] for d in detections]}")
                return None

            # Get current robot pose (as 4x4 matrix)
            robot_pose = self.robot.get_pose_matrix()

            print(f"✅ Pose {pose_index}: Detected {len(valid_detections)} tags")

            return robot_pose, valid_detections

        except Exception as e:
            print(f"❌ Error capturing calibration data: {e}")
            return None

    def collect_calibration_dataset(self, poses: List[np.ndarray]) -> bool:
        """Collect hand-eye calibration dataset.

        Args:
            poses: List of robot poses for calibration

        Returns:
            True if dataset collection successful
        """
        print(f"🤖 Collecting hand-eye calibration dataset with {len(poses)} poses...")

        successful_captures = 0

        for i, pose in enumerate(poses):
            print(f"\n📍 Moving to calibration pose {i + 1}/{len(poses)}")

            # Move to pose
            if not self.move_to_pose_safely(pose):
                continue

            # Capture calibration data
            result = self.capture_calibration_data(i + 1)
            if result is None:
                continue

            robot_pose, tag_detections = result

            # Store data (using first valid tag for now)
            first_tag_id = list(tag_detections.keys())[0]
            tag_detection = tag_detections[first_tag_id]

            # Convert tag pose to transformation matrix
            tag_pose = self._tag_detection_to_transform(tag_detection)

            self.robot_poses.append(robot_pose)
            self.tag_poses.append(tag_pose)

            successful_captures += 1

        print(f"\n✅ Successfully collected {successful_captures} pose pairs")

        if successful_captures < 10:
            print("⚠️  Warning: Less than 10 poses collected. Calibration may be inaccurate.")

        return successful_captures >= 8  # Minimum viable dataset

    def _tag_detection_to_transform(self, detection: Dict) -> np.ndarray:
        """Convert AprilTag detection to 4x4 transformation matrix.

        Args:
            detection: AprilTag detection dictionary

        Returns:
            4x4 transformation matrix
        """
        # Extract pose information from AprilTag detection format
        if 'pose' in detection and detection['pose'] is not None:
            tvec = np.array(detection['pose']['translation_vector']).flatten()
            rvec = np.array(detection['pose']['rotation_vector']).flatten()
        else:
            raise ValueError(f"No pose information found in detection: {detection.keys()}")

        # Create transformation matrix
        transform = np.eye(4)
        transform[:3, 3] = tvec
        transform[:3, :3] = R.from_rotvec(rvec).as_matrix()

        return transform

    def solve_hand_eye_calibration(self) -> bool:
        """Solve hand-eye calibration using collected data.

        Uses the classic AX = XB formulation for eye-in-hand calibration.

        Returns:
            True if calibration successful
        """
        if len(self.robot_poses) < 8:
            print("❌ Insufficient data for calibration (need at least 8 poses)")
            return False

        print(f"🔬 Solving hand-eye calibration with {len(self.robot_poses)} pose pairs...")

        try:
            # Use OpenCV's hand-eye calibration
            # For eye-in-hand: solves for camera pose in end-effector frame

            # Convert to OpenCV format
            R_gripper2base = [pose[:3, :3] for pose in self.robot_poses]
            t_gripper2base = [pose[:3, 3] for pose in self.robot_poses]
            R_target2cam = [pose[:3, :3] for pose in self.tag_poses]
            t_target2cam = [pose[:3, 3] for pose in self.tag_poses]

            # Solve hand-eye calibration
            R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
                R_gripper2base, t_gripper2base,
                R_target2cam, t_target2cam,
                method=cv2.CALIB_HAND_EYE_TSAI
            )

            # Create 4x4 transformation matrix
            self.hand_eye_transform = np.eye(4)
            self.hand_eye_transform[:3, :3] = R_cam2gripper
            self.hand_eye_transform[:3, 3] = t_cam2gripper.flatten()

            # Apply coordinate frame correction for UR robot convention
            # OpenCV camera frame: X-right, Y-down, Z-forward
            # UR robot TCP frame: X-forward, Y-left, Z-up
            # Transformation from OpenCV camera frame to UR TCP frame
            frame_correction = np.array([
                [0, 0, 1],   # OpenCV Z (forward) -> UR X (forward)
                [-1, 0, 0],  # OpenCV -X (left) -> UR Y (left)
                [0, -1, 0]   # OpenCV -Y (up) -> UR Z (up)
            ])

            # Apply frame correction to rotation
            corrected_rotation = frame_correction @ R_cam2gripper

            # Apply frame correction to translation
            corrected_translation = frame_correction @ t_cam2gripper.flatten()

            # Update transform with corrections
            self.hand_eye_transform[:3, :3] = corrected_rotation
            self.hand_eye_transform[:3, 3] = corrected_translation

            # Compute residuals for validation
            self._compute_calibration_residuals()

            print("✅ Hand-eye calibration completed!")
            self._print_calibration_results()

            return True

        except Exception as e:
            print(f"❌ Hand-eye calibration failed: {e}")
            return False

    def _compute_calibration_residuals(self):
        """Compute calibration residuals for validation."""
        self.calibration_residuals = []

        for i, (robot_pose, tag_pose) in enumerate(zip(self.robot_poses, self.tag_poses)):
            # Expected tag pose using hand-eye calibration
            expected_tag_pose = np.linalg.inv(self.hand_eye_transform) @ tag_pose

            # Actual tag pose
            actual_tag_pose = tag_pose

            # Compute pose error
            pose_error = np.linalg.inv(actual_tag_pose) @ expected_tag_pose

            # Extract translation and rotation errors
            trans_error = np.linalg.norm(pose_error[:3, 3]) * 1000  # mm
            rot_error = np.rad2deg(np.linalg.norm(R.from_matrix(pose_error[:3, :3]).as_rotvec()))

            self.calibration_residuals.append({
                'pose_index': i + 1,
                'translation_error_mm': trans_error,
                'rotation_error_deg': rot_error
            })

    def _print_calibration_results(self):
        """Print calibration results and residuals."""
        print("\n📊 Hand-Eye Calibration Results:")
        print("Transform Matrix (Camera to End-Effector):")
        print(self.hand_eye_transform)

        print("\n📏 Calibration Residuals:")
        avg_trans_error = np.mean([r['translation_error_mm'] for r in self.calibration_residuals])
        avg_rot_error = np.mean([r['rotation_error_deg'] for r in self.calibration_residuals])

        print(f"Average Translation Error: {avg_trans_error:.2f} mm")
        print(f"Average Rotation Error: {avg_rot_error:.2f} degrees")

        print("\nPer-pose residuals:")
        for residual in self.calibration_residuals:
            print(f"  Pose {residual['pose_index']}: "
                  f"{residual['translation_error_mm']:.2f}mm, "
                  f"{residual['rotation_error_deg']:.2f}°")

        # Quality assessment
        if avg_trans_error < 5.0 and avg_rot_error < 2.0:
            print("✅ Excellent calibration quality!")
        elif avg_trans_error < 10.0 and avg_rot_error < 5.0:
            print("✅ Good calibration quality")
        else:
            print("⚠️  Calibration quality may be poor. Consider recalibrating.")

    def save_calibration(self, filepath: Path):
        """Save hand-eye calibration to file.

        Args:
            filepath: Path to save calibration file
        """
        if self.hand_eye_transform is None:
            print("❌ No calibration to save")
            return

        calibration_data = {
            'hand_eye_transform': self.hand_eye_transform.tolist(),
            'calibration_date': datetime.datetime.now().isoformat(),
            'num_poses': len(self.robot_poses),
            'residuals': self.calibration_residuals,
            'tag_ids': self.tag_ids
        }

        filepath.parent.mkdir(parents=True, exist_ok=True)

        with open(filepath, 'w') as f:
            json.dump(calibration_data, f, indent=2)

        print(f"💾 Calibration saved to: {filepath}")

    def load_calibration(self, filepath: Path) -> bool:
        """Load hand-eye calibration from file.

        Args:
            filepath: Path to calibration file

        Returns:
            True if loaded successfully
        """
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)

            self.hand_eye_transform = np.array(data['hand_eye_transform'])
            self.calibration_residuals = data.get('residuals', [])

            print(f"📂 Calibration loaded from: {filepath}")
            print(f"   Date: {data.get('calibration_date', 'Unknown')}")
            print(f"   Poses used: {data.get('num_poses', 'Unknown')}")

            return True

        except Exception as e:
            print(f"❌ Failed to load calibration: {e}")
            return False

    def run_manual_calibration(self, num_poses: int = 15) -> bool:
        """Run manual hand-eye calibration with freedrive.

        Args:
            num_poses: Number of poses to collect

        Returns:
            True if calibration successful
        """
        print("🤖 Manual Hand-Eye Calibration")
        print("=" * 40)
        print(f"Target poses: {num_poses}")
        print("\n📝 Instructions:")
        print("1. Use freedrive to position the robot")
        print("2. Ensure AprilTag is visible in camera view")
        print("3. Press Enter to capture data at each pose")
        print("4. Type 'q' to finish early")

        # Enable freedrive mode
        print("\n🔓 Enabling freedrive mode...")
        self.robot.enable_freedrive()

        print("✅ Freedrive enabled! Move the robot manually.")

        successful_captures = 0

        for i in range(num_poses):
            print(f"\n📍 Pose {i + 1}/{num_poses}")
            print("Position robot for AprilTag visibility...")

            # Wait for user input
            user_input = input("Press Enter to capture (or 'q' to quit): ").strip().lower()
            if user_input == 'q':
                print("🛑 Calibration stopped by user")
                break

            # Capture data at current pose
            result = self.capture_calibration_data(i)
            if result is not None:
                robot_pose, valid_detections = result
                self.robot_poses.append(robot_pose)
                # Use the first (and typically only) detected tag
                first_detection = list(valid_detections.values())[0]
                self.tag_poses.append(self._tag_detection_to_transform(first_detection))
                successful_captures += 1
                print(f"✅ Captured pose {i + 1} - Total: {successful_captures}")
            else:
                print(f"❌ Failed to capture pose {i + 1}")

        # Disable freedrive
        print("\n🔒 Disabling freedrive mode...")
        self.robot.disable_freedrive()

        if successful_captures < 4:
            print(f"❌ Insufficient data: {successful_captures} poses (need at least 4)")
            return False

        print(f"\n🎯 Collected {successful_captures} valid poses")

        # Solve calibration
        return self.solve_hand_eye_calibration()

    def run_automatic_calibration(self, num_poses: int = 15) -> bool:
        """Run complete automatic hand-eye calibration.

        Args:
            num_poses: Number of poses to use for calibration

        Returns:
            True if calibration successful
        """
        print("🚀 Starting automatic hand-eye calibration...")

        try:
            # Generate calibration poses
            poses = self.generate_calibration_poses(num_poses)

            # Collect dataset
            if not self.collect_calibration_dataset(poses):
                print("❌ Failed to collect sufficient calibration data")
                return False

            # Solve calibration
            if not self.solve_hand_eye_calibration():
                print("❌ Failed to solve hand-eye calibration")
                return False

            # Save calibration
            calib_file = Path("src/ur_toolkit/hand_eye_calibration/hand_eye_calibration.json")
            self.save_calibration(calib_file)

            print("🎉 Hand-eye calibration completed successfully!")
            return True

        except Exception as e:
            print(f"❌ Calibration failed: {e}")
            return False
