#!/usr/bin/env python3
"""
Simple Pi Camera Test with Config File
Tests connection and captures a photo using the centralized config.yaml or command-line arguments
"""

import argparse
import sys
from pathlib import Path

# Add src directory to path for ur_toolkit imports
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from ur_toolkit.camera.picam.picam import PiCam, PiCamConfig
from ur_toolkit.config_manager import get_camera_host, get_camera_port


def main():
    parser = argparse.ArgumentParser(description="Test Pi Camera connection and capture")
    parser.add_argument("--host", help="Camera server hostname/IP (overrides config file)")
    parser.add_argument("--port", type=int, help="Camera server port (overrides config file)")
    args = parser.parse_args()

    print("🍓 Pi Camera Client Test")
    print("========================")

    if args.host or args.port:
        # Use command-line arguments (with fallbacks from config)
        host = args.host or get_camera_host()
        port = args.port or get_camera_port()
        print("📝 Using command-line arguments...")
        print(f"🔗 Connecting to: {host}:{port}")
        config = PiCamConfig(hostname=host, port=port)
        cam = PiCam(config)
    else:
        # Load config from centralized config.yaml
        print("📝 Loading config from centralized config.yaml...")
        host = get_camera_host()
        port = get_camera_port()
        print(f"🔗 Connecting to: {host}:{port}")
        config = PiCamConfig(hostname=host, port=port)
        cam = PiCam(config)

    # Test connection
    print("\n🔍 Testing connection...")
    if cam.test_connection():
        print("✅ Connection successful!")
    else:
        print("❌ Connection failed!")
        print("   Check your Pi's IP address in config.yaml")
        print("   Make sure the camera server is running on the Pi")
        return

    # Capture photo
    print("\n📸 Capturing photo...")
    photo_path = cam.capture_photo()

    if photo_path:
        print("✅ Photo captured successfully!")
        print(f"📁 Saved to: {photo_path}")
    else:
        print("❌ Failed to capture photo")


if __name__ == "__main__":
    main()
