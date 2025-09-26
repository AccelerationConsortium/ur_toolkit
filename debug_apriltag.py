#!/usr/bin/env python3
"""
Debug AprilTag Detection
Test AprilTag detection on a captured image
"""

import sys
from pathlib import Path
import cv2

# Add src directory to path for ur_toolkit imports
sys.path.insert(0, str(Path(__file__).parent / "src"))

from ur_toolkit.apriltag_detection import AprilTagDetector
from ur_toolkit.camera.picam.picam import PiCam, PiCamConfig
from ur_toolkit.config_manager import get_camera_host, get_camera_port


def main():
    print("🏷️  AprilTag Detection Debug")
    print("============================")

    # Initialize camera and capture photo
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
    
    # Initialize AprilTag detector
    print("🏷️  Initializing AprilTag detector...")
    detector = AprilTagDetector()
    print(f"   Family: {detector.tag_family}")
    print(f"   Tag size: {detector.tag_size}m")
    
    # Load and detect tags
    print("🔍 Loading image and detecting tags...")
    image = cv2.imread(str(photo_path))
    
    if image is None:
        print("❌ Failed to load image")
        return
        
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
            print(f"     Detection data: {detection}")
            if isinstance(detection, dict):
                print(f"     ID: {detection.get('tag_id', 'unknown')}")
                print(f"     Family: {detection.get('tag_family', 'unknown')}")
                print(f"     Center: {detection.get('center', 'unknown')}")
            else:
                print(f"     Type: {type(detection)}")
                print(f"     Attributes: {dir(detection)}")
            
    # Save annotated image
    annotated_image = detector.draw_detections(image, detections)
    output_path = Path(photo_path).parent / f"debug_apriltag_{Path(photo_path).stem}.jpg"
    cv2.imwrite(str(output_path), annotated_image)
    print(f"💾 Annotated image saved to: {output_path}")


if __name__ == "__main__":
    main()