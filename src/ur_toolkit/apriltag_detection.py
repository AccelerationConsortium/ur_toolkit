#!/usr/bin/env python3
"""
AprilTag Detection and Pose Estimation
Requires calibrated camera for accurate pose estimation
"""

import cv2
import numpy as np
import yaml
import argparse
from datetime import datetime
import time
from pathlib import Path

from ur_toolkit.config_manager import (config, get_apriltag_family, get_apriltag_size,
                                       get_camera_calibration_file)
from ur_toolkit.camera.picam.picam import PiCam, PiCamConfig

try:
    from pupil_apriltags import Detector as PupilAprilTagDetector
    APRILTAG_AVAILABLE = True
except ImportError:
    APRILTAG_AVAILABLE = False
    print("⚠️  pupil-apriltags library not installed")
    print("   Install with: pip install pupil-apriltags")


class AprilTagDetector:
    """AprilTag detection and pose estimation"""

    def __init__(self, tag_family=None, tag_size=None, calibration_file=None):
        """
        Initialize AprilTag detector

        Args:
            tag_family: AprilTag family (overrides config if provided)
            tag_size: Physical size of AprilTag in meters
                (overrides config if provided)
            calibration_file: Path to camera calibration YAML file
                (overrides config if provided)
        """
        if not APRILTAG_AVAILABLE:
            raise ImportError("pupil-apriltags library not installed")

        self.tag_family = tag_family or get_apriltag_family()
        self.tag_size = tag_size or get_apriltag_size()

        if calibration_file is None:
            self.calibration_file = get_camera_calibration_file()
        else:
            self.calibration_file = Path(calibration_file)
            if not self.calibration_file.is_absolute():
                self.calibration_file = config.resolve_path(calibration_file)

        print(f"🏷️  AprilTag Detector: {self.tag_family}, "
              f"size={self.tag_size}m")
        print(f"📹 Camera calibration: {self.calibration_file}")

        # Initialize detector with better error handling
        try:
            # pupil-apriltags expects families as a string, not a list
            if not self.tag_family or not isinstance(self.tag_family, str):
                raise ValueError(f"Invalid tag family: {self.tag_family}")

            self.detector = PupilAprilTagDetector(families=self.tag_family)
            print(f"✅ AprilTag detector initialized with family: {self.tag_family}")
        except Exception as e:
            print(f"❌ Failed to initialize AprilTag detector: {e}")
            print(f"   Tag family: {self.tag_family} (type: {type(self.tag_family)})")
            raise

        # Load camera calibration if provided
        self.camera_matrix = None
        self.dist_coeffs = None
        self.pose_estimation_enabled = False

        if self.calibration_file and self.calibration_file.exists():
            self.load_calibration(self.calibration_file)
        else:
            print(f"⚠️  Camera calibration file not found: {self.calibration_file}")
            print("   Pose estimation will be disabled")

    def load_calibration(self, calibration_file):
        """Load camera calibration from YAML file"""
        try:
            with open(calibration_file, 'r') as f:
                # Try safe_load first, fall back to unsafe load for compatibility
                try:
                    calib_data = yaml.safe_load(f)
                except yaml.constructor.ConstructorError:
                    # Reset file pointer and try unsafe load for legacy files with tuples
                    f.seek(0)
                    calib_data = yaml.load(f, Loader=yaml.FullLoader)

            self.camera_matrix = np.array(calib_data['camera_matrix'])
            self.dist_coeffs = np.array(calib_data['distortion_coefficients'])
            self.pose_estimation_enabled = True

            print(f"✅ Loaded camera calibration from: {calibration_file}")
            print(f"   Focal length: fx={self.camera_matrix[0, 0]:.1f}, fy={self.camera_matrix[1, 1]:.1f}")

        except Exception as e:
            print(f"⚠️  Failed to load calibration: {e}")
            print("   Pose estimation disabled")

    def detect_tags(self, image, estimate_pose=True):
        """
        Detect AprilTags in image

        Args:
            image: Input image (BGR or grayscale)
            estimate_pose: Whether to estimate 6DOF pose (requires calibration)

        Returns:
            list: Detected tags with corners, pose, and metadata
        """
        # Convert to grayscale if needed
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image

        # Detect tags using pupil-apriltags
        tags = self.detector.detect(gray, estimate_tag_pose=False)

        results = []
        for tag in tags:
            result = {
                'tag_id': tag.tag_id,
                'family': tag.tag_family,
                'corners': tag.corners,
                'center': tag.center,
                'hamming': tag.hamming,
                'decision_margin': tag.decision_margin,
                'pose': None,
                'distance': None,
                'distance_mm': None
            }

            # Estimate pose if calibration available
            if (estimate_pose and self.pose_estimation_enabled
                    and tag.hamming == 0):  # Only use perfect detections for pose

                pose = self.estimate_pose(tag.corners)
                if pose is not None:
                    result['pose'] = pose

                    # Calculate distance (translation magnitude)
                    tvec = pose['translation_vector']
                    distance = np.linalg.norm(tvec)
                    result['distance'] = float(distance)
                    result['distance_mm'] = float(distance * 1000)  # For display compatibility

            results.append(result)

        return results

    def estimate_pose(self, corners):
        """
        Estimate 6DOF pose from tag corners

        Args:
            corners: 4x2 array of corner coordinates

        Returns:
            dict: Pose information (rotation, translation)
        """
        if not self.pose_estimation_enabled:
            return None

        # Define 3D object points for AprilTag corners
        # Standard AprilTag: corners at +/-0.5 * tag_size from center
        half_size = self.tag_size / 2.0
        object_points = np.array([
            [-half_size, -half_size, 0],  # Top-left
            [half_size, -half_size, 0],  # Top-right
            [half_size, half_size, 0],  # Bottom-right
            [-half_size, half_size, 0],  # Bottom-left
        ], dtype=np.float32)

        # Image points (tag corners)
        image_points = corners.astype(np.float32)

        # Solve PnP
        success, rvec, tvec = cv2.solvePnP(
            object_points, image_points,
            self.camera_matrix, self.dist_coeffs
        )

        if not success:
            return None

        # Convert rotation vector to rotation matrix
        rmat, _ = cv2.Rodrigues(rvec)

        return {
            'rotation_vector': rvec.flatten().tolist(),
            'translation_vector': tvec.flatten().tolist(),
            'rotation_matrix': rmat.tolist(),
            'success': True
        }

    def draw_detections(self, image, detections, draw_pose=True, draw_id=True):
        """
        Draw detected tags on image

        Args:
            image: Input image
            detections: List of detection results
            draw_pose: Whether to draw pose axes (requires pose data)
            draw_id: Whether to draw tag IDs

        Returns:
            annotated image
        """
        annotated = image.copy()

        for detection in detections:
            corners = detection['corners'].astype(int)
            center = detection['center'].astype(int)
            tag_id = detection['tag_id']

            # Draw tag outline
            cv2.polylines(annotated, [corners], True, (0, 255, 0), 2)

            # Draw center point
            cv2.circle(annotated, tuple(center), 5, (0, 0, 255), -1)

            # Draw tag ID
            if draw_id:
                cv2.putText(annotated, f"ID: {tag_id}",
                            tuple(corners[0]), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (255, 255, 255), 2)
                cv2.putText(annotated, f"ID: {tag_id}",
                            tuple(corners[0]), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 0, 0), 1)

            # Draw pose axes if available
            if draw_pose and detection['pose'] is not None:
                self.draw_pose_axes(annotated, detection['pose'])

            # Draw distance if available
            if detection['distance_mm'] is not None:
                dist_text = f"{detection['distance_mm']:.1f}mm"
                cv2.putText(annotated, dist_text,
                            (center[0] - 30, center[1] + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(annotated, dist_text,
                            (center[0] - 30, center[1] + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        return annotated

    def draw_pose_axes(self, image, pose, axis_length=30):
        """Draw 3D coordinate axes for pose visualization"""
        if not self.pose_estimation_enabled:
            return

        # Define 3D axes points
        axes_points = np.array([
            [0, 0, 0],                    # Origin
            [axis_length, 0, 0],          # X-axis (red)
            [0, axis_length, 0],          # Y-axis (green)
            [0, 0, -axis_length],         # Z-axis (blue)
        ], dtype=np.float32)

        # Project to image
        rvec = np.array(pose['rotation_vector'])
        tvec = np.array(pose['translation_vector'])

        projected, _ = cv2.projectPoints(
            axes_points, rvec, tvec,
            self.camera_matrix, self.dist_coeffs
        )

        projected = projected.reshape(-1, 2).astype(int)
        origin = tuple(projected[0])

        # Draw axes
        cv2.arrowedLine(image, origin, tuple(projected[1]), (0, 0, 255), 3)  # X-axis: red
        cv2.arrowedLine(image, origin, tuple(projected[2]), (0, 255, 0), 3)  # Y-axis: green
        cv2.arrowedLine(image, origin, tuple(projected[3]), (255, 0, 0), 3)  # Z-axis: blue


def main():
    parser = argparse.ArgumentParser(description='AprilTag Detection and Pose Estimation')
    parser.add_argument('--host', help='Camera server hostname/IP (overrides config)')
    parser.add_argument('--port', type=int, help='Camera server port (overrides config)')
    parser.add_argument('--calibration', help='Camera calibration file (overrides config)')
    parser.add_argument('--tag-family', choices=['tag36h11', 'tag25h9', 'tag16h5'],
                        help='AprilTag family (overrides config)')
    parser.add_argument('--tag-size', type=float,
                        help='AprilTag size in millimeters (overrides config)')
    parser.add_argument('--save-detections', action='store_true',
                        help='Save images with detections')
    parser.add_argument('--continuous', action='store_true',
                        help='Continuous detection mode')

    args = parser.parse_args()

    print("🏷️  AprilTag Detection and Pose Estimation")
    print("=" * 60)

    if not APRILTAG_AVAILABLE:
        return

    print("📡 Connecting to Pi camera...")
    if args.host or args.port:
        host = args.host or config.get('camera.server.host')
        port = args.port or config.get('camera.server.port')
        print(f"🔗 Using: {host}:{port}")
        camera_config = PiCamConfig(hostname=host, port=port)
        camera = PiCam(camera_config)
    else:
        host = config.get('camera.server.host')
        port = config.get('camera.server.port')
        timeout = config.get('camera.server.timeout', 10)
        print(f"🔗 Using config: {host}:{port}")
        camera_config = PiCamConfig(hostname=host, port=port, timeout=timeout)
        camera = PiCam(camera_config)

    if not camera.test_connection():
        print("❌ Failed to connect to camera server")
        return

    print("✅ Connected to camera server")

    # Initialize detector with config/argument values
    tag_family = args.tag_family or get_apriltag_family()
    tag_size_mm = args.tag_size or (get_apriltag_size() * 1000)  # Convert m to mm
    calibration_file = args.calibration or get_camera_calibration_file()

    detector = AprilTagDetector(
        tag_family=tag_family,
        tag_size=tag_size_mm / 1000.0,  # Convert mm to m
        calibration_file=calibration_file
    )

    print("🏷️  Detector initialized:")
    print(f"   Family: {tag_family}")
    print(f"   Tag size: {tag_size_mm}mm")
    print(f"   Pose estimation: {'✅' if detector.pose_estimation_enabled else '❌'}")
    print("")

    if args.continuous:
        print("🔄 Continuous detection mode - Press Ctrl+C to stop")
        try:
            while True:
                # Capture image
                photo_path = camera.capture_photo()
                if not photo_path:
                    print("❌ Failed to capture image")
                    time.sleep(1)
                    continue

                # Load and detect
                image = cv2.imread(photo_path)
                if image is None:
                    continue

                detections = detector.detect_tags(image)

                if detections:
                    print(f"🏷️  Detected {len(detections)} tags:")
                    for det in detections:
                        pose_info = ""
                        if det['distance_mm']:
                            pose_info = f" @ {det['distance_mm']:.1f}mm"
                        print(f"   ID {det['tag_id']}: quality={det['decision_margin']:.2f}{pose_info}")
                else:
                    print("   No tags detected")

                time.sleep(0.5)

        except KeyboardInterrupt:
            print("\n🛑 Stopped by user")

    else:
        print("📸 Single detection mode")
        input("Press ENTER to capture and detect...")

        # Capture image
        photo_path = camera.capture_photo()
        if not photo_path:
            print("❌ Failed to capture image")
            return

        # Load and detect
        image = cv2.imread(photo_path)
        if image is None:
            print("❌ Failed to load image")
            return

        print("🔍 Detecting AprilTags...")
        detections = detector.detect_tags(image)

        if detections:
            print(f"✅ Detected {len(detections)} AprilTags:")
            for i, det in enumerate(detections):
                print(f"\nTag {i + 1}:")
                print(f"   ID: {det['tag_id']}")
                print(f"   Family: {det['family']}")
                print(f"   Detection Quality: {det['decision_margin']:.3f}")
                print(f"   Hamming Distance: {det['hamming']} (0=perfect, 1-2=good, >2=poor)")

                if det['pose']:
                    tvec = det['pose']['translation_vector']

                    print("   📍 Position (camera frame):")
                    print(f"     X: {tvec[0]:+7.1f}mm (left-/right+)")
                    print(f"     Y: {tvec[1]:+7.1f}mm (up-/down+)")
                    print(f"     Z: {tvec[2]:+7.1f}mm (forward+/back-)")
                    print(f"   📏 Distance: {det['distance_mm']:.1f}mm")

                    # Convert rotation vector to Euler angles for easier interpretation
                    rmat = np.array(det['pose']['rotation_matrix'])

                    # Extract Euler angles (in degrees)
                    sy = np.sqrt(rmat[0, 0] * rmat[0, 0] + rmat[1, 0] * rmat[1, 0])
                    singular = sy < 1e-6

                    if not singular:
                        x = np.arctan2(rmat[2, 1], rmat[2, 2])
                        y = np.arctan2(-rmat[2, 0], sy)
                        z = np.arctan2(rmat[1, 0], rmat[0, 0])
                    else:
                        x = np.arctan2(-rmat[1, 2], rmat[1, 1])
                        y = np.arctan2(-rmat[2, 0], sy)
                        z = 0

                    # Convert to degrees
                    roll = np.degrees(x)
                    pitch = np.degrees(y)
                    yaw = np.degrees(z)

                    print("   🔄 Orientation (degrees):")
                    print(f"     Roll:  {roll:+6.1f}° (rotation around X-axis)")
                    print(f"     Pitch: {pitch:+6.1f}° (rotation around Y-axis)")
                    print(f"     Yaw:   {yaw:+6.1f}° (rotation around Z-axis)")

                    # Quality assessment based on detection confidence
                    if det['hamming'] == 0 and det['decision_margin'] > 50:
                        quality = "Excellent - High precision pose"
                    elif det['hamming'] == 0 and det['decision_margin'] > 20:
                        quality = "Good - Reliable pose"
                    elif det['hamming'] <= 1 and det['decision_margin'] > 10:
                        quality = "Fair - Acceptable pose"
                    else:
                        quality = "Poor - Use with caution"

                    print(f"   ⭐ Pose Quality: {quality}")
                else:
                    print("   ❌ No pose estimation (missing calibration or poor detection)")
        else:
            print("❌ No AprilTags detected")

        # Save annotated image
        if args.save_detections or detections:
            annotated = detector.draw_detections(image, detections)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_path = f"apriltag_detection_{timestamp}.jpg"
            cv2.imwrite(output_path, annotated)
            print(f"💾 Saved annotated image: {output_path}")


if __name__ == "__main__":
    main()
