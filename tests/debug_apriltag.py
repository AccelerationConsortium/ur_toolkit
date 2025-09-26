#!/usr/bin/env python3
"""
Debug AprilTag Detection
Test AprilTag detection on captured or saved images
"""

import sys
import argparse
from pathlib import Path
import cv2
import glob

# Add src directory to path for ur_toolkit imports
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from ur_toolkit.apriltag_detection import AprilTagDetector
from ur_toolkit.camera.picam.picam import PiCam, PiCamConfig
from ur_toolkit.config_manager import get_camera_host, get_camera_port


def process_image(image_path, detector, source_description=""):
    """Process a single image for AprilTag detection"""
    print(f"🔍 Processing image: {Path(image_path).name} {source_description}")
    
    # Load image
    image = cv2.imread(str(image_path))
    if image is None:
        print("❌ Failed to load image")
        return False
        
    print(f"   Image size: {image.shape[1]}x{image.shape[0]}")
    
    # Detect tags
    detections = detector.detect_tags(image)
    
    print(f"🏷️  Found {len(detections)} tags:")
    
    if len(detections) == 0:
        print("   No tags detected")
        print("   Possible issues:")
        print("   - Tag not visible in image")
        print("   - Tag too small/large")
        print("   - Poor lighting")
        print("   - Wrong tag family (looking for tag36h11)")
    else:
        for i, detection in enumerate(detections):
            print(f"   Tag {i+1}:")
            print(f"     ID: {detection.get('tag_id', 'unknown')}")
            print(f"     Family: {detection.get('tag_family', 'unknown')}")
            print(f"     Quality: {detection.get('decision_margin', 'unknown')}")
            print(f"     Hamming: {detection.get('hamming', 'unknown')}")
            
            if detection.get('pose') and detection.get('distance_mm'):
                distance = detection['distance_mm']
                tvec = detection['pose']['translation_vector']
                print(f"     Distance: {distance:.1f}mm")
                print(f"     Position: [{tvec[0]:.3f}, {tvec[1]:.3f}, {tvec[2]:.3f}]")
            
    # Save annotated image
    annotated_image = detector.draw_detections(image, detections)
    output_path = Path(image_path).parent / f"debug_apriltag_{Path(image_path).stem}.jpg"
    cv2.imwrite(str(output_path), annotated_image)
    print(f"💾 Annotated image saved to: {output_path.name}")
    
    return True


def main():
    parser = argparse.ArgumentParser(description='Debug AprilTag detection')
    parser.add_argument('--photo', help='Path to saved photo file (if not provided, captures new photo)')
    parser.add_argument('--photos', nargs='+', help='Process multiple photos or glob pattern (e.g., photos/*.jpg)')
    
    args = parser.parse_args()
    
    print("🏷️  AprilTag Detection Debug")
    print("============================")

    # Initialize AprilTag detector
    print("🏷️  Initializing AprilTag detector...")
    detector = AprilTagDetector()
    print(f"   Family: {detector.tag_family}")
    print(f"   Tag size: {detector.tag_size}m")
    print(f"   Pose estimation: {'✅' if detector.pose_estimation_enabled else '❌'}")
    
    # Determine input mode
    if args.photos:
        # Process multiple photos/glob patterns
        photo_paths = []
        for pattern in args.photos:
            if '*' in pattern or '?' in pattern:
                matches = glob.glob(pattern)
                if not matches:
                    print(f"⚠️  No files match pattern: {pattern}")
                else:
                    photo_paths.extend(matches)
            else:
                if Path(pattern).exists():
                    photo_paths.append(pattern)
                else:
                    print(f"⚠️  File not found: {pattern}")
        
        if not photo_paths:
            print("❌ No valid photo files found")
            return
            
        print(f"\n📁 Processing {len(photo_paths)} saved photo(s)...")
        for i, photo_path in enumerate(photo_paths):
            print(f"\n--- Photo {i+1}/{len(photo_paths)} ---")
            process_image(photo_path, detector, "(saved)")
            
    elif args.photo:
        # Process single saved photo
        if not Path(args.photo).exists():
            print(f"❌ Photo file not found: {args.photo}")
            return
            
        print("📁 Processing saved photo...")
        process_image(args.photo, detector, "(saved)")
        
    else:
        # Capture new photo from camera
        host = get_camera_host()
        port = get_camera_port()
        config = PiCamConfig(hostname=host, port=port)
        camera = PiCam(config)
        
        print("📸 Capturing new photo...")
        photo_path = camera.capture_photo()
        
        if not photo_path:
            print("❌ Failed to capture photo")
            return
            
        print(f"✅ Photo saved to: {photo_path}")
        process_image(photo_path, detector, "(captured)")


if __name__ == "__main__":
    main()