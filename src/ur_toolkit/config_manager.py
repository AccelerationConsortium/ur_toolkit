#!/usr/bin/env python3
"""
Central Configuration Manager for Robot System Tools
Provides unified configuration access for all system components
"""

import yaml
import os
from pathlib import Path
from typing import Dict, Any, Optional


class ConfigManager:
    """Centralized configuration manager for the robot system"""

    _instance = None
    _config = None

    def __new__(cls):
        """Singleton pattern to ensure one config instance"""
        if cls._instance is None:
            cls._instance = super(ConfigManager, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        """Initialize configuration manager"""
        if self._config is None:
            self.reload_config()

    @staticmethod
    def find_project_root():
        """Find project root by looking for marker files"""
        current = Path(__file__).parent  # Start from setup directory

        # Since config.yaml is in setup/, we need to look for project-level markers
        project_markers = ['apriltag_detection.py', 'README.md', '.git']

        while current != current.parent:
            # Check parent directory for project markers
            if any((current.parent / marker).exists() for marker in project_markers):
                return current.parent
            current = current.parent

        # If not found, assume parent of setup directory is project root
        return Path(__file__).parent.parent

    def get_config_path(self, config_file: str = "config.yaml") -> Path:
        """Get path to configuration file"""
        if 'ROBOT_TOOLS_CONFIG' in os.environ:
            config_path = Path(os.environ['ROBOT_TOOLS_CONFIG'])
            if config_path.exists():
                return config_path

        project_root = self.find_project_root()
        # First try the new setup directory structure
        config_path = project_root / "setup" / config_file

        if config_path.exists():
            return config_path

        # Fallback to old config directory for backward compatibility
        config_path = project_root / "config" / config_file
        if config_path.exists():
            return config_path

        # Fallback to old location for backward compatibility
        config_path = project_root / config_file
        if config_path.exists():
            return config_path

        fallback_path = Path(__file__).parent / config_file
        return fallback_path

    def load_config(self, config_file: str = "config.yaml") -> Dict[str, Any]:
        """Load configuration from YAML file"""
        config_path = self.get_config_path(config_file)

        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)

            config['_project_root'] = self.find_project_root()

            print(f"✅ Loaded configuration from: {config_path}")
            return config

        except FileNotFoundError:
            print(f"⚠️  Config file not found: {config_path}")
            return self.get_default_config()
        except yaml.YAMLError as e:
            print(f"⚠️  Error parsing config file: {e}")
            return self.get_default_config()
        except Exception as e:
            print(f"⚠️  Error loading config: {e}")
            return self.get_default_config()

    def get_default_config(self) -> Dict[str, Any]:
        """Get default configuration if config file is not available"""
        return {
            'system': {
                'name': 'Robot System Tools',
                'version': '1.0',
                'environment': 'development'
            },
            'robot': {
                'type': 'UR',
                'ip_address': '192.168.0.10',
                'default_speed': 0.03,
                'default_acceleration': 0.08
            },
            'camera': {
                'type': 'pi_camera',
                'server': {
                    'host': '192.168.1.100',
                    'port': 2222,
                    'timeout': 10
                },
                'client': {
                    'download_directory': 'photos'
                }
            },
            'apriltag': {
                'family': 'tag36h11',
                'tag_size': 0.023
            },
            '_project_root': self.find_project_root()
        }

    def reload_config(self, config_file: str = "config.yaml"):
        """Reload configuration from file"""
        self._config = self.load_config(config_file)

    def get(self, key_path: str, default: Any = None) -> Any:
        """Get configuration value using dot notation"""
        if self._config is None:
            self.reload_config()

        keys = key_path.split('.')
        value = self._config

        try:
            for key in keys:
                value = value[key]
            return value
        except (KeyError, TypeError):
            return default

    def get_section(self, section: str) -> Dict[str, Any]:
        """Get entire configuration section"""
        return self.get(section, {})

    def resolve_path(self, relative_path: str) -> Path:
        """Resolve relative path to absolute path based on project root"""
        project_root = self.get('_project_root')

        # Special handling for positions files
        if relative_path == 'taught_positions.yaml':
            # First try new src structure
            src_positions_path = project_root / "src" / "ur_toolkit" / "positions" / relative_path
            if src_positions_path.exists():
                return src_positions_path
            # Try old positions directory for backwards compatibility
            positions_path = project_root / "positions" / relative_path
            if positions_path.exists():
                return positions_path
            # Also try setup directory for backwards compatibility
            setup_path = project_root / "setup" / relative_path
            if setup_path.exists():
                return setup_path
            # Default to src positions directory even if file doesn't exist yet
            return src_positions_path

        return project_root / relative_path

    def get_robot_config(self) -> Dict[str, Any]:
        """Get robot configuration section"""
        return self.get_section('robot')

    def get_camera_config(self) -> Dict[str, Any]:
        """Get camera configuration section"""
        return self.get_section('camera')

    def get_apriltag_config(self) -> Dict[str, Any]:
        """Get AprilTag configuration section"""
        return self.get_section('apriltag')

    def get_paths_config(self) -> Dict[str, Any]:
        """Get paths configuration section"""
        return self.get_section('paths')

    def print_config(self, section: Optional[str] = None):
        """Print configuration for debugging"""
        if section:
            config_to_print = self.get_section(section)
            print(f"\n📋 Configuration section '{section}':")
        else:
            config_to_print = self._config
            print("\n📋 Full configuration:")

        import json
        print(json.dumps(config_to_print, indent=2, default=str))


# Global configuration instance
config = ConfigManager()


def get_robot_ip() -> str:
    """Get robot IP address"""
    return config.get('robot.ip_address', '192.168.0.10')


def get_robot_speed() -> float:
    """Get default robot speed"""
    return config.get('robot.default_speed', 0.03)


def get_camera_host() -> str:
    """Get camera server host"""
    return config.get('camera.server.host', '192.168.1.100')


def get_camera_port() -> int:
    """Get camera server port"""
    return config.get('camera.server.port', 2222)


def get_apriltag_family() -> str:
    """Get AprilTag family"""
    return config.get('apriltag.family', 'tag36h11')


def get_apriltag_size() -> float:
    """Get AprilTag physical size in meters"""
    return config.get('apriltag.tag_size', 0.023)


def get_camera_calibration_file() -> Path:
    """Get path to camera calibration file"""
    calib_file = config.get('camera.calibration.file', 'camera_calibration/camera_calibration.yaml')
    return config.resolve_path(calib_file)


def get_photos_directory() -> Path:
    """Get path to photos directory"""
    photos_dir = config.get('camera.client.download_directory', 'photos')
    return config.resolve_path(photos_dir)


def main():
    """Test configuration loading"""
    print("🔧 Testing Configuration Manager")
    print("=" * 50)

    # Test basic access
    print(f"Robot IP: {get_robot_ip()}")
    print(f"Robot Speed: {get_robot_speed()}")
    print(f"Camera Host: {get_camera_host()}")
    print(f"AprilTag Family: {get_apriltag_family()}")
    print(f"AprilTag Size: {get_apriltag_size()}")

    # Test path resolution
    print(f"Camera Calibration File: {get_camera_calibration_file()}")
    print(f"Photos Directory: {get_photos_directory()}")

    # Print robot section
    config.print_config('robot')


if __name__ == "__main__":
    main()
