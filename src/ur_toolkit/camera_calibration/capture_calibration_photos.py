#!/usr/bin/env python3
"""
Camera Calibration Photo Capture
Captures calibration photos using chessboard pattern for camera intrinsics calculation
"""

import cv2
import sys
import argparse
from pathlib import Path
from datetime import datetime

# Add parent directory to path to import picam
sys.path.append(str(Path(__file__).parent.parent))
sys.path.append(str(Path(__file__).parent.parent / 'camera' / 'picam'))
from picam import PiCam, PiCamConfig


class CalibrationPhotoCapture:
    """Capture calibration photos using chessboard pattern"""

    def __init__(self, checkerboard_size=(8, 6), square_size_mm=30.0):
        """
        Initialize photo capture for calibration

        Args:
            checkerboard_size: (width, height) - number of internal corners
                              For 8x6 external corners = 7x5 internal corners
            square_size_mm: Size of checkerboard squares in millimeters
        """
        self.checkerboard_size = checkerboard_size
        self.square_size_mm = square_size_mm

        print("📐 Calibration Setup:")
        print(f"   Chessboard: {checkerboard_size[0] + 1}x{checkerboard_size[1] + 1} external corners")
        print(f"   Internal corners: {checkerboard_size[0]}x{checkerboard_size[1]}")
        print(f"   Square size: {square_size_mm}mm")

    def capture_calibration_photos(self, pi_camera, num_photos=10, save_dir="calibration_photos"):
        """
        Capture calibration photos using Pi camera

        Args:
            pi_camera: PiCam instance
            num_photos: Number of calibration photos to capture
            save_dir: Directory to save calibration photos
        """
        save_path = Path(save_dir)
        save_path.mkdir(exist_ok=True)

        # Create subdirectory for failed validation images
        failed_path = save_path / "failed_validation"
        failed_path.mkdir(exist_ok=True)

        print("\n📸 Starting calibration photo capture")
        print(f"   Target: {num_photos} photos")
        print(f"   Save directory: {save_path}")
        print(f"   Failed validation images: {failed_path}")
        print("")
        print("📋 Instructions:")
        print("1. Print 'Calibration chessboard (US Letter).pdf' at 100% scale")
        print("2. Mount on rigid surface (cardboard/clipboard)")
        print(f"3. Verify square size is {self.square_size_mm}mm after printing")
        print("4. Hold pattern at different positions and angles")
        print("5. Press ENTER to capture when pattern is clearly visible")
        print("6. Ensure good lighting and all corners are visible")
        print("")

        captured_valid = 0
        total_attempts = 0

        while captured_valid < num_photos:
            input(f"📸 Position chessboard for photo {captured_valid + 1}/{num_photos}. Press ENTER to capture...")

            # Capture image
            print("📸 Capturing image...")
            photo_path = pi_camera.capture_photo()

            if not photo_path:
                print("❌ Failed to capture image. Try again.")
                continue

            total_attempts += 1

            # Load and process image
            img = cv2.imread(photo_path)
            if img is None:
                print("❌ Failed to load captured image. Try again.")
                continue

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Find checkerboard corners
            print("🔍 Detecting chessboard corners...")
            ret, corners = cv2.findChessboardCorners(gray, self.checkerboard_size, None)

            # Generate timestamp for this capture
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

            if ret:
                # Save only the clean original image with simple filename
                clean_image_path = save_path / f"calib_{captured_valid:02d}.jpg"
                cv2.imwrite(str(clean_image_path), img)

                captured_valid += 1
                print(f"✅ Valid chessboard detected! Saved: {clean_image_path.name}")
                print(f"   Progress: {captured_valid}/{num_photos} photos captured")

            else:
                # Save failed validation image for review
                failed_image_path = failed_path / f"failed_{total_attempts:02d}_{timestamp}.jpg"
                cv2.imwrite(str(failed_image_path), img)

                print("❌ No chessboard detected. Try:")
                print("   - Better lighting")
                print("   - Different angle/distance")
                print("   - Ensure entire chessboard is visible")
                print("   - Check focus")
                print("   - Make sure pattern is flat")
                print("   - Verify you're using the correct chessboard size")
                print(f"   💾 Failed image saved: {failed_image_path.name} (review and delete later)")

            print("")

        print("🎉 Calibration photo capture complete!")
        print(f"   Valid photos: {captured_valid}")
        print(f"   Total attempts: {total_attempts}")
        print(f"   Success rate: {captured_valid / total_attempts * 100:.1f}%")
        print(f"   Photos saved in: {save_path}")

        # Check if there are failed images
        failed_images = list(failed_path.glob("failed_*.jpg"))
        if failed_images:
            print(f"   Failed validation images: {len(failed_images)} in {failed_path}")
            print(f"   💡 Review failed images and delete directory when done: rm -rf {failed_path}")

        print("")
        print("📝 Next step: Run 'python calculate_camera_intrinsics.py' to compute calibration")

        return captured_valid


def main():
    parser = argparse.ArgumentParser(description='Capture Camera Calibration Photos')
    parser.add_argument('--config', default='../client_config.yaml',
                        help='Pi camera config file (relative to parent directory)')
    parser.add_argument('--photos', type=int, default=10,
                        help='Number of calibration photos to capture')
    parser.add_argument('--square-size', type=float, default=30.0,
                        help='Chessboard square size in mm (verify after printing)')
    parser.add_argument('--save-dir', default='calibration_photos',
                        help='Directory to save calibration photos')

    args = parser.parse_args()

    print("🍓 Pi Camera Calibration - Photo Capture")
    print("=" * 60)

    # Check if chessboard PDF exists
    chessboard_pdf = Path("Calibration chessboard (US Letter).pdf")
    if chessboard_pdf.exists():
        print(f"✅ Calibration chessboard found: {chessboard_pdf}")
    else:
        print(f"❌ Calibration chessboard not found: {chessboard_pdf}")
        print("   Make sure you're running from the camera_calibration directory")
        return

    # Load camera config and connect
    config_path = Path(args.config)
    if not config_path.exists():
        print(f"❌ Config file not found: {config_path}")
        print("   Update the --config path or copy client_config.yaml")
        return

    print(f"📡 Connecting to Pi camera using: {config_path}")
    config = PiCamConfig.from_yaml(str(config_path))
    camera = PiCam(config)

    if not camera.test_connection():
        print("❌ Failed to connect to camera server")
        print("   Check Pi IP in config file and ensure server is running")
        return

    print("✅ Connected to camera server")

    # Initialize photo capture
    # For 8x6 external corners = 7x5 internal corners
    capturer = CalibrationPhotoCapture(
        checkerboard_size=(7, 5),  # 8x6 external = 7x5 internal
        square_size_mm=args.square_size
    )

    try:
        # Capture calibration photos
        num_captured = capturer.capture_calibration_photos(
            camera, args.photos, args.save_dir
        )

        if num_captured < 8:
            print(f"⚠️  Only {num_captured} photos captured.")
            print("   Recommended: At least 8-10 photos for good calibration")
            print("   Run script again to capture more photos")

    except KeyboardInterrupt:
        print("\n🛑 Photo capture cancelled by user")
    except Exception as e:
        print(f"❌ Photo capture failed: {e}")


if __name__ == "__main__":
    main()
