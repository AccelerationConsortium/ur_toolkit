#!/usr/bin/env python3
"""
Robust Pose Estimator using OpenCV solvePnP
Provides stable pose estimation to replace direct AprilTag pose calculation
"""

import cv2
import numpy as np
from typing import Optional, Tuple
from scipy.spatial.transform import Rotation as R


class RobustPoseEstimator:
    """Robust pose estimation using OpenCV solvePnP with RANSAC"""
    
    def __init__(self, camera_matrix: np.ndarray, dist_coeffs: np.ndarray, tag_size: float = 0.05):
        """
        Initialize robust pose estimator
        
        Args:
            camera_matrix: Camera intrinsic matrix (3x3)
            dist_coeffs: Distortion coefficients
            tag_size: Physical size of AprilTag in meters
        """
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.tag_size = tag_size
        
        # Define 3D tag corners in tag coordinate frame (Z=0 plane)
        self.tag_corners_3d = np.array([
            [-tag_size/2, -tag_size/2, 0],
            [ tag_size/2, -tag_size/2, 0], 
            [ tag_size/2,  tag_size/2, 0],
            [-tag_size/2,  tag_size/2, 0]
        ], dtype=np.float32)
        
        print(f"🎯 Robust Pose Estimator initialized (tag_size={tag_size}m)")
    
    def estimate_pose_ransac(self, detection) -> Optional[Tuple[np.ndarray, np.ndarray, int]]:
        """
        Estimate pose using robust RANSAC approach
        
        Args:
            detection: AprilTag detection (dict with 'corners' key or object with .corners attribute)
            
        Returns:
            Tuple of (rvec, tvec, num_inliers) or None if estimation failed
        """
        try:
            # Extract 2D corners from detection (handle both dict and object formats)
            if isinstance(detection, dict):
                corners_2d = detection['corners'].reshape(-1, 2).astype(np.float32)
            else:
                corners_2d = detection.corners.reshape(-1, 2).astype(np.float32)
            
            # Use solvePnPRansac for robust estimation
            success, rvec, tvec, inliers = cv2.solvePnPRansac(
                self.tag_corners_3d, corners_2d,
                self.camera_matrix, self.dist_coeffs,
                iterationsCount=500,
                reprojectionError=2.0,
                confidence=0.95,
                flags=cv2.SOLVEPNP_IPPE_SQUARE  # Optimal for planar squares
            )
            
            if not success or inliers is None or len(inliers) < 3:
                print("⚠️  RANSAC failed, trying standard pose estimation...")
                # Fallback to standard pose estimation
                fallback_result = self.estimate_pose_standard(detection)
                if fallback_result is not None:
                    rvec_fb, tvec_fb = fallback_result
                    print("✅ Standard pose estimation succeeded as fallback")
                    return rvec_fb, tvec_fb, 4  # Assume all corners used
                else:
                    print("❌ Both RANSAC and standard pose estimation failed")
                    return None
                
            num_inliers = len(inliers)
            print(f"✅ Robust pose estimated with {num_inliers}/4 inliers")
            
            return rvec, tvec, num_inliers
            
        except Exception as e:
            print(f"❌ Pose estimation error: {e}")
            return None
    
    def estimate_pose_standard(self, detection) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        Estimate pose using standard solvePnP (faster, less robust)
        
        Args:
            detection: AprilTag detection object with corners
            
        Returns:
            Tuple of (rvec, tvec) or None if estimation failed
        """
        try:
            # Extract 2D corners from detection (handle both dict and object formats)
            if isinstance(detection, dict):
                corners_2d = detection['corners'].reshape(-1, 2).astype(np.float32)
            else:
                corners_2d = detection.corners.reshape(-1, 2).astype(np.float32)
            
            success, rvec, tvec = cv2.solvePnP(
                self.tag_corners_3d, corners_2d,
                self.camera_matrix, self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            
            if not success:
                return None
                
            return rvec, tvec
            
        except Exception as e:
            print(f"❌ Standard pose estimation error: {e}")
            return None
    
    def pose_to_6dof(self, rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
        """
        Convert rotation vector and translation to 6DOF pose
        
        Args:
            rvec: Rotation vector (3x1)
            tvec: Translation vector (3x1)
            
        Returns:
            6DOF pose [x, y, z, rx, ry, rz]
        """
        # Extract translation
        translation = tvec.flatten()
        
        # Convert rotation vector to euler angles
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        rotation = R.from_matrix(rotation_matrix)
        euler_angles = rotation.as_euler('xyz', degrees=False)
        
        return np.concatenate([translation, euler_angles])
    
    def pose_to_matrix(self, rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
        """
        Convert rotation vector and translation to 4x4 transformation matrix
        
        Args:
            rvec: Rotation vector (3x1)
            tvec: Translation vector (3x1)
            
        Returns:
            4x4 transformation matrix
        """
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        
        transform = np.eye(4)
        transform[:3, :3] = rotation_matrix
        transform[:3, 3] = tvec.flatten()
        
        return transform
    
    def validate_pose(self, rvec: np.ndarray, tvec: np.ndarray, 
                     max_distance: float = 1.0, max_rotation: float = np.pi) -> bool:
        """
        Validate that pose estimate is reasonable
        
        Args:
            rvec: Rotation vector
            tvec: Translation vector  
            max_distance: Maximum allowed distance from camera (meters)
            max_rotation: Maximum allowed rotation (radians)
            
        Returns:
            True if pose is valid
        """
        try:
            # Check distance
            distance = np.linalg.norm(tvec)
            if distance > max_distance:
                print(f"⚠️  Pose validation failed: distance {distance:.3f}m > {max_distance}m")
                return False
            
            # Check rotation magnitude
            rotation_magnitude = np.linalg.norm(rvec)
            if rotation_magnitude > max_rotation:
                print(f"⚠️  Pose validation failed: rotation {rotation_magnitude:.3f}rad > {max_rotation:.3f}rad")
                return False
                
            return True
            
        except Exception as e:
            print(f"❌ Pose validation error: {e}")
            return False