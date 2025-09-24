#!/usr/bin/env python3
"""
Calculate Camera Intrinsics
Processes calibration photos to calculate camera intrinsic parameters
"""

import cv2
import numpy as np
import yaml
import glob
from pathlib import Path
from datetime import datetime


class CameraIntrinsicsCalculator:
    """Calculate camera intrinsics from calibration photos"""

    def __init__(self, checkerboard_size=(7, 5), square_size_mm=30.0):
        """
        Initialize intrinsics calculator

        Args:
            checkerboard_size: (width, height) - number of internal corners
            square_size_mm: Size of checkerboard squares in millimeters
        """
        self.checkerboard_size = checkerboard_size
        self.square_size_mm = square_size_mm

        # Prepare object points (0,0,0), (1,0,0), (2,0,0) ....,(6,4,0)
        self.objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)
        self.objp *= square_size_mm  # Convert to real world coordinates

        # Arrays to store object points and image points
        self.objpoints = []  # 3d points in real world space
        self.imgpoints = []  # 2d points in image plane

        self.image_size = None

    def process_calibration_photos(self, photo_dir="calibration_photos"):
        """
        Process calibration photos to extract corner points

        Args:
            photo_dir: Directory containing calibration photos

        Returns:
            int: Number of valid photos processed
        """
        photo_path = Path(photo_dir)
        if not photo_path.exists():
            raise FileNotFoundError(f"Photo directory not found: {photo_path}")

        # Find all calibration photos
        photo_patterns = [
            "calib_*.jpg",      # New simple naming format
            "original_*.jpg",   # Legacy format (if any exist)
            "calibration_*.jpg"  # Legacy format (if any exist)
        ]

        photo_files = []
        for pattern in photo_patterns:
            photo_files.extend(glob.glob(str(photo_path / pattern)))

        # Remove duplicates and sort
        photo_files = sorted(list(set(photo_files)))

        if not photo_files:
            raise FileNotFoundError(f"No calibration photos found in {photo_path}")

        print(f"🔍 Processing {len(photo_files)} calibration photos...")
        print(f"   Checkerboard: {self.checkerboard_size[0]}x{self.checkerboard_size[1]} internal corners")
        print(f"   Square size: {self.square_size_mm}mm")
        print("")

        valid_photos = 0

        for i, photo_file in enumerate(photo_files):
            print(f"📸 Processing {Path(photo_file).name}... ", end="")

            # Load image
            img = cv2.imread(photo_file)
            if img is None:
                print("❌ Failed to load")
                continue

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Find checkerboard corners
            ret, corners = cv2.findChessboardCorners(gray, self.checkerboard_size, None)

            if ret:
                # Refine corner positions
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

                # Store image points and object points
                self.imgpoints.append(corners2)
                self.objpoints.append(self.objp)

                if self.image_size is None:
                    self.image_size = gray.shape[::-1]

                valid_photos += 1
                print("✅ Valid")

            else:
                print("❌ No corners detected")

        print("\n📊 Processing complete:")
        print(f"   Total photos: {len(photo_files)}")
        print(f"   Valid photos: {valid_photos}")
        print(f"   Success rate: {valid_photos / len(photo_files) * 100:.1f}%")

        return valid_photos

    def calculate_intrinsics(self):
        """
        Calculate camera intrinsics using processed photos

        Returns:
            dict: Calibration results containing camera matrix, distortion coefficients, etc.
        """
        if len(self.objpoints) < 8:
            raise ValueError(f"Need at least 8 valid photos, got {len(self.objpoints)}")

        print("\n🔬 Calculating camera intrinsics...")
        print(f"   Using {len(self.objpoints)} photos")
        print(f"   Image size: {self.image_size}")

        # Perform calibration
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            self.objpoints, self.imgpoints, self.image_size, None, None
        )

        if not ret:
            raise RuntimeError("Camera calibration failed")

        # Calculate reprojection error
        total_error = 0
        for i in range(len(self.objpoints)):
            imgpoints2, _ = cv2.projectPoints(self.objpoints[i], rvecs[i], tvecs[i],
                                              camera_matrix, dist_coeffs)
            error = cv2.norm(self.imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
            total_error += error

        mean_error = total_error / len(self.objpoints)

        # Prepare results
        results = {
            'calibration_successful': True,
            'camera_matrix': camera_matrix.tolist(),
            'distortion_coefficients': dist_coeffs.tolist(),
            'image_size': self.image_size,
            'reprojection_error': float(mean_error),
            'num_images_used': len(self.objpoints),
            'checkerboard_size': self.checkerboard_size,
            'square_size_mm': self.square_size_mm,
            'calibration_date': datetime.now().isoformat(),

            # Camera parameters for convenience
            'focal_length_x': float(camera_matrix[0, 0]),
            'focal_length_y': float(camera_matrix[1, 1]),
            'principal_point_x': float(camera_matrix[0, 2]),
            'principal_point_y': float(camera_matrix[1, 2]),

            # Quality assessment
            'calibration_quality': self.assess_quality(mean_error)
        }

        print("✅ Calibration successful!")
        print(f"   Reprojection error: {mean_error:.3f} pixels")
        print(f"   Quality: {results['calibration_quality']}")
        print(f"   Focal length: fx={results['focal_length_x']:.1f}, fy={results['focal_length_y']:.1f}")
        print(f"   Principal point: ({results['principal_point_x']:.1f}, {results['principal_point_y']:.1f})")

        return results

    def assess_quality(self, reprojection_error):
        """Assess calibration quality based on reprojection error"""
        if reprojection_error < 0.3:
            return "Excellent (<0.3px) - High precision poses, sub-millimeter accuracy"
        elif reprojection_error < 0.5:
            return "Very Good (<0.5px) - Good precision poses, ~1-2mm accuracy"
        elif reprojection_error < 1.0:
            return "Good (<1.0px) - Acceptable poses, ~2-5mm accuracy"
        elif reprojection_error < 2.0:
            return "Fair (<2.0px) - Lower precision, ~5-10mm accuracy"
        else:
            return "Poor (>2.0px) - Unreliable poses, >10mm error - recalibrate"

    def save_intrinsics(self, results, output_file="camera_calibration.yaml"):
        """
        Save calibration results to YAML file

        Args:
            results: Calibration results dictionary
            output_file: Filename to save the calibration file (in current directory)
        """
        output_path = Path(output_file)

        # Convert any tuples to lists for safe YAML serialization
        clean_results = {}
        for key, value in results.items():
            if isinstance(value, tuple):
                clean_results[key] = list(value)
            elif isinstance(value, np.ndarray):
                clean_results[key] = value.tolist()
            else:
                clean_results[key] = value

        with open(output_path, 'w') as f:
            yaml.dump(clean_results, f, default_flow_style=False, indent=2)

        print(f"💾 Camera intrinsics saved to: {output_path}")
        print("   This file can be used for AprilTag pose estimation")

        return output_path


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Calculate Camera Intrinsics from Calibration Photos')
    parser.add_argument('--photo-dir', default='calibration_photos',
                        help='Directory containing calibration photos')
    parser.add_argument('--square-size', type=float, default=30.0,
                        help='Chessboard square size in mm')
    parser.add_argument('--output-file', default='camera_calibration.yaml',
                        help='Filename to save camera_calibration.yaml')

    args = parser.parse_args()

    print("🔬 Camera Intrinsics Calculation")
    print("=" * 50)

    try:
        # Initialize calculator for 8x6 external corners = 7x5 internal corners
        calculator = CameraIntrinsicsCalculator(
            checkerboard_size=(7, 5),
            square_size_mm=args.square_size
        )

        # Process calibration photos
        valid_photos = calculator.process_calibration_photos(args.photo_dir)

        if valid_photos < 8:
            print(f"\n❌ Insufficient valid photos: {valid_photos}")
            print("   Need at least 8 photos for reliable calibration")
            print("   Capture more photos with 'python capture_calibration_photos.py'")
            return

        # Calculate intrinsics
        results = calculator.calculate_intrinsics()

        # Save results
        output_path = calculator.save_intrinsics(results, args.output_file)

        print("\n🎉 Camera calibration complete!")
        print(f"   Calibration file: {output_path}")
        print(f"   Reprojection error: {results['reprojection_error']:.3f} pixels")
        print(f"   Quality: {results['calibration_quality']}")
        print("\n📝 Ready for AprilTag pose estimation!")
        print(f"   Use: python ../test_apriltag_detection.py --calibration camera_calibration/{output_path.name}")

    except Exception as e:
        print(f"❌ Calibration failed: {e}")
        print("\n🔧 Troubleshooting:")
        print("   1. Ensure calibration photos are in the correct directory")
        print("   2. Check that chessboard was detected in photos")
        print("   3. Verify square size matches printed chessboard")
        print("   4. Recapture photos if needed")


if __name__ == "__main__":
    main()
