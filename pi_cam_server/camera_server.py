#!/usr/bin/env python3
"""
Pi Camera Server - Standalone Application
Minimal camera server for Raspberry Pi deployment
"""

import socket
import threading
import time
import os
import yaml
import logging
import argparse
from pathlib import Path
from datetime import datetime

try:
    from picamera2 import Picamera2
    CAMERA_AVAILABLE = True
except ImportError:
    print("❌ picamera2 not available")
    CAMERA_AVAILABLE = False

class SimpleCameraConfig:
    """Simple camera configuration"""
    def __init__(self):
        self.server_port = 2222
        self.photo_directory = "photos"
        self.focus_mode = "auto"
        self.image_format = "jpeg"
        self.image_quality = 85
        self.rotation = 0
        self.resolution = [1920, 1080]
        self.hflip = False
        self.vflip = False
        self.brightness = 0
        self.contrast = 0
        self.saturation = 0

    @classmethod
    def from_yaml(cls, config_path):
        """Load config from YAML file"""
        config = cls()
        try:
            with open(config_path, 'r') as f:
                data = yaml.safe_load(f)
                if data:
                    # Load server settings
                    if 'server' in data:
                        server = data['server']
                        config.server_port = server.get('port', config.server_port)
                        config.photo_directory = server.get('photo_directory', config.photo_directory)

                    # Load camera settings
                    if 'camera' in data:
                        camera = data['camera']
                        config.rotation = camera.get('rotation', config.rotation)
                        config.image_format = camera.get('image_format', config.image_format)
                        config.image_quality = camera.get('image_quality', config.image_quality)
                        config.resolution = camera.get('resolution', config.resolution)
                        config.focus_mode = camera.get('focus_mode', config.focus_mode)

                    # Load advanced settings
                    if 'advanced' in data:
                        advanced = data['advanced']
                        config.hflip = advanced.get('hflip', config.hflip)
                        config.vflip = advanced.get('vflip', config.vflip)
                        config.brightness = advanced.get('brightness', config.brightness)
                        config.contrast = advanced.get('contrast', config.contrast)
                        config.saturation = advanced.get('saturation', config.saturation)

        except Exception as e:
            print(f"⚠️  Could not load config from {config_path}: {e}")
        return config

class SimpleCameraServer:
    """Simple camera server for Pi"""

    def __init__(self, config=None):
        self.config = config or SimpleCameraConfig()
        self.server_socket = None
        self.running = False

        # Ensure photo directory exists
        Path(self.config.photo_directory).mkdir(exist_ok=True)

        # Initialize camera
        self.camera = None
        if CAMERA_AVAILABLE:
            try:
                print("🔍 Initializing camera...")
                self.camera = Picamera2()

                # Test camera with basic configuration
                config = self.camera.create_still_configuration()
                self.camera.configure(config)
                print("✅ Camera initialized successfully")

            except Exception as e:
                print(f"❌ Camera initialization failed: {e}")
                print("💡 Troubleshooting tips:")
                print("   1. Check camera is enabled: sudo raspi-config → Interface Options → Camera → Enable")
                print("   2. Check camera connection (ribbon cable)")
                print("   3. Reboot Pi: sudo reboot")
                print("   4. Check no other process using camera: sudo fuser /dev/video*")
                print("   5. Test camera manually: rpicam-still -o test.jpg")
                self.camera = None
        else:
            print("❌ picamera2 not available")
            print("💡 Install with: sudo apt install python3-picamera2")

    def take_photo(self) -> str:
        """Take a photo and return the filename"""
        if not self.camera:
            raise Exception("Camera not available")

        timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        filename = f"capture_{timestamp}.jpg"
        filepath = Path(self.config.photo_directory) / filename

        try:
            # Create camera configuration with rotation and settings
            config = self.camera.create_still_configuration(
                main={"size": tuple(self.config.resolution)}
            )

            # Apply rotation if specified
            if self.config.rotation != 0:
                try:
                    import libcamera
                    transform = libcamera.Transform()

                    # Apply rotation
                    if self.config.rotation == 90:
                        transform = libcamera.Transform(hflip=False, vflip=True, transpose=True)
                    elif self.config.rotation == 180:
                        transform = libcamera.Transform(hflip=True, vflip=True)
                    elif self.config.rotation == 270:
                        transform = libcamera.Transform(hflip=True, vflip=False, transpose=True)

                    config["transform"] = transform
                except ImportError:
                    print("⚠️  libcamera not available for rotation, using software rotation")

            # Apply flip settings
            if self.config.hflip or self.config.vflip:
                try:
                    import libcamera
                    if "transform" not in config:
                        config["transform"] = libcamera.Transform()
                    # Note: Additional flip logic would go here if needed
                except ImportError:
                    pass

            self.camera.configure(config)
            self.camera.start()

            # Apply camera controls if available
            try:
                controls = {}
                if hasattr(self.config, 'brightness') and self.config.brightness != 0:
                    controls["Brightness"] = self.config.brightness / 100.0
                if hasattr(self.config, 'contrast') and self.config.contrast != 0:
                    controls["Contrast"] = 1.0 + (self.config.contrast / 100.0)
                if hasattr(self.config, 'saturation') and self.config.saturation != 0:
                    controls["Saturation"] = 1.0 + (self.config.saturation / 100.0)

                if controls:
                    self.camera.set_controls(controls)
            except Exception as e:
                print(f"⚠️  Could not apply camera controls: {e}")

            # Capture image
            self.camera.capture_file(str(filepath))
            self.camera.stop()

            print(f"📸 Photo captured: {filename} (rotation: {self.config.rotation}°)")
            return str(filepath)

        except Exception as e:
            print(f"❌ Photo capture failed: {e}")
            raise

    def handle_client(self, client_socket, client_address):
        """Handle client connection"""
        try:
            print(f"📱 Client connected: {client_address}")

            # Receive command
            command = client_socket.recv(1024).decode().strip()
            print(f"📋 Command: {command}")

            if command == "CAPTURE":
                try:
                    photo_path = self.take_photo()

                    # Send photo file
                    with open(photo_path, 'rb') as f:
                        photo_data = f.read()

                    # Send response header
                    response = f"OK {len(photo_data)}\n".encode()
                    client_socket.send(response)

                    # Send photo data
                    client_socket.send(photo_data)
                    print(f"📤 Sent {len(photo_data)} bytes")

                except Exception as e:
                    error_msg = f"ERROR {str(e)}\n".encode()
                    client_socket.send(error_msg)
                    print(f"❌ Error: {e}")

            elif command == "TEST":
                response = "OK Camera server ready\n".encode()
                client_socket.send(response)
                print("✅ Test response sent")

            else:
                response = "ERROR Unknown command\n".encode()
                client_socket.send(response)
                print(f"❌ Unknown command: {command}")

        except Exception as e:
            print(f"❌ Client handling error: {e}")
        finally:
            client_socket.close()
            print(f"📱 Client disconnected: {client_address}")

    def start_server(self):
        """Start the camera server"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind(('0.0.0.0', self.config.server_port))
            self.server_socket.listen(5)

            self.running = True
            print(f"🎥 Camera server listening on port {self.config.server_port}")
            print("   Ready for connections...")

            while self.running:
                try:
                    client_socket, client_address = self.server_socket.accept()
                    client_thread = threading.Thread(
                        target=self.handle_client,
                        args=(client_socket, client_address)
                    )
                    client_thread.daemon = True
                    client_thread.start()

                except Exception as e:
                    if self.running:
                        print(f"❌ Accept error: {e}")

        except Exception as e:
            print(f"❌ Server start error: {e}")
        finally:
            self.stop_server()

    def stop_server(self):
        """Stop the camera server"""
        self.running = False
        if self.server_socket:
            self.server_socket.close()
        if self.camera:
            self.camera.close()
        print("🛑 Camera server stopped")

def setup_logging(log_level: str = "INFO"):
    """Setup logging for Pi deployment"""
    log_dir = Path("logs")
    log_dir.mkdir(exist_ok=True)

    logging.basicConfig(
        level=getattr(logging, log_level.upper()),
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler(log_dir / "camera_server.log"),
            logging.StreamHandler()
        ]
    )

def load_config():
    """Load configuration for Pi server"""
    config_paths = [
        "camera_config.yaml",
        "../camera_config.yaml",
        "/etc/camera_server/config.yaml",
        "/home/ac/pi_camera_server/camera_config.yaml"
    ]

    for config_path in config_paths:
        if os.path.exists(config_path):
            print(f"📝 Loading config from: {config_path}")
            return SimpleCameraConfig.from_yaml(config_path)

    print("📝 Using default configuration")
    return SimpleCameraConfig()

def main():
    """Main function"""
    parser = argparse.ArgumentParser(description="Pi Camera Server")
    parser.add_argument("--port", type=int, default=2222, help="Server port")
    parser.add_argument("--log-level", default="INFO", help="Log level")
    parser.add_argument("--daemon", action="store_true", help="Run as daemon")

    args = parser.parse_args()

    # Setup logging
    setup_logging(args.log_level)

    # Load configuration
    config = load_config()
    if args.port != 2222:
        config.server_port = args.port

    # Create and start server
    server = SimpleCameraServer(config)

    if not args.daemon:
        print("🎥 Pi Camera Server Starting")
        print("=" * 40)
        print(f"📡 Port: {config.server_port}")
        print(f"📁 Photos: {config.photo_directory}")
        print("📸 Ready to serve camera captures")
        print("Press Ctrl+C to stop")
        print()

    try:
        server.start_server()
    except KeyboardInterrupt:
        print("\n🛑 Stopping server...")
        server.stop_server()

if __name__ == "__main__":
    main()
