#!/usr/bin/env python3
"""Quick script to detect available AprilTags"""
import sys
from pathlib import Path
import cv2

sys.path.insert(0, str(Path(__file__).parent / "src"))
sys.path.insert(0, str(Path(__file__).parent / "setup"))

from ur_toolkit.camera.picam.picam import PiCam, PiCamConfig
from ur_toolkit.apriltag_detection import AprilTagDetector
from ur_toolkit.config_manager import get_apriltag_family, get_apriltag_size, get_camera_calibration_file, config

# Initialize camera
host = config.get('camera.server.host')
port = config.get('camera.server.port')
camera_config = PiCamConfig(hostname=host, port=port)
camera = PiCam(camera_config)

# Initialize detector
detector = AprilTagDetector(
    tag_family=get_apriltag_family(),
    tag_size=get_apriltag_size(),
    calibration_file=get_camera_calibration_file()
)

print('📷 Capturing image to detect available AprilTags...')
photo_path = camera.capture_photo()
if photo_path:
    print(f'📁 Image saved: {photo_path}')
    image = cv2.imread(photo_path)
    if image is not None:
        detections = detector.detect_tags(image)
        if detections:
            print(f'✅ Found {len(detections)} AprilTag(s):')
            for det in detections:
                print(f'   - Tag ID {det["tag_id"]}, quality: {det["decision_margin"]:.2f}')
        else:
            print('❌ No AprilTags detected')
            print('   Make sure AprilTag is visible and well-lit')
    else:
        print('❌ Failed to load image')
else:
    print('❌ Failed to capture image')