#!/usr/bin/env python3
"""
Simple Pi Camera Class
Focused camera functionality for robot vision scripts
"""

import socket
import os
import yaml
from datetime import datetime
from pathlib import Path
from typing import Optional


class PiCamConfig:
    """Simple camera configuration"""
    def __init__(self, hostname="raspberrypi.local", port=2222, download_dir="photos"):
        self.hostname = hostname
        self.port = port
        self.download_dir = download_dir
        self.timeout = 10

        # Create download directory
        Path(self.download_dir).mkdir(exist_ok=True)

    @classmethod
    def from_yaml(cls, config_path: str = "camera_client_config.yaml"):
        """Load config from client YAML file"""
        if not os.path.exists(config_path):
            print(f"⚠️  Config file {config_path} not found, using defaults")
            return cls()

        with open(config_path, 'r') as f:
            data = yaml.safe_load(f)

        # Extract server config
        server_config = data.get('server', {})
        client_config = data.get('client', {})

        return cls(
            hostname=server_config.get('host', 'raspberrypi.local'),
            port=server_config.get('port', 2222),
            download_dir=client_config.get('download_directory', 'photos')
        )


class PiCam:
    """
    Simple Pi Camera Client

    Provides basic camera operations for robot vision scripts:
    - Connect to Pi camera server
    - Capture photos
    - Test connections
    """

    def __init__(self, config: Optional[PiCamConfig] = None):
        """Initialize camera client"""
        self.config = config or PiCamConfig()

    def test_connection(self) -> bool:
        """Test if camera server is accessible"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(self.config.timeout)
            result = sock.connect_ex((self.config.hostname, self.config.port))
            sock.close()
            return result == 0
        except Exception:
            return False

    def get_status(self) -> str:
        """Get detailed camera server status"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(self.config.timeout)
            sock.connect((self.config.hostname, self.config.port))

            # Send status command
            sock.send("STATUS".encode('utf-8'))

            # Receive response
            response = sock.recv(1024).decode().strip()
            sock.close()
            return response

        except Exception as e:
            return f"Connection failed: {e}"

    def capture_photo(self, filename: Optional[str] = None) -> Optional[str]:
        """
        Capture a photo from Pi camera server

        Args:
            filename: Optional custom filename

        Returns:
            Path to saved photo or None if failed
        """
        try:
            # Connect to server
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(self.config.timeout)
            sock.connect((self.config.hostname, self.config.port))

            # Send capture command
            sock.send("CAPTURE".encode('utf-8'))

            # Receive response header (read until newline)
            response_bytes = b""
            while b"\n" not in response_bytes:
                chunk = sock.recv(1)
                if not chunk:
                    break
                response_bytes += chunk

            response = response_bytes.decode('utf-8').strip()
            if not response.startswith("OK"):
                print(f"Server error: {response}")
                sock.close()
                return None

            # Parse data length from response "OK {length}"
            try:
                data_length = int(response.split()[1])
            except (IndexError, ValueError):
                print("Invalid server response format")
                sock.close()
                return None

            # Generate filename if not provided
            if filename:
                file_path = Path(self.config.download_dir) / filename
            else:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                file_path = Path(self.config.download_dir) / f"capture_{timestamp}.jpg"

            # Receive photo data
            received_size = 0
            with open(file_path, 'wb') as f:
                while received_size < data_length:
                    remaining = data_length - received_size
                    chunk_size = min(4096, remaining)
                    chunk = sock.recv(chunk_size)

                    if not chunk:
                        break

                    f.write(chunk)
                    received_size += len(chunk)

            sock.close()

            if received_size == data_length:
                return str(file_path)
            else:
                print(f"Incomplete download: {received_size}/{data_length} bytes")
                return None

        except Exception as e:
            print(f"Photo capture failed: {e}")
            return None

    def get_latest_photo(self) -> Optional[str]:
        """Get the path to the latest photo in download directory"""
        photo_dir = Path(self.config.download_dir)
        if not photo_dir.exists():
            return None

        photo_files = list(photo_dir.glob("*.jpg")) + list(photo_dir.glob("*.jpeg"))
        if not photo_files:
            return None

        latest = max(photo_files, key=os.path.getctime)
        return str(latest)


# Convenience functions for quick usage
def capture_photo(hostname: str = "raspberrypi.local", port: int = 2222) -> Optional[str]:
    """Quick photo capture function"""
    config = PiCamConfig(hostname, port)
    camera = PiCam(config)
    return camera.capture_photo()


def test_camera(hostname: str = "raspberrypi.local", port: int = 2222) -> bool:
    """Quick connection test function"""
    config = PiCamConfig(hostname, port)
    camera = PiCam(config)
    return camera.test_connection()


# Example usage
if __name__ == "__main__":
    # Simple test
    camera = PiCam()

    print("Testing camera connection...")
    if camera.test_connection():
        print("✅ Connected to camera server")

        print("Capturing photo...")
        photo_path = camera.capture_photo()
        if photo_path:
            print(f"📸 Photo saved: {photo_path}")
        else:
            print("❌ Failed to capture photo")
    else:
        print("❌ Cannot connect to camera server")
