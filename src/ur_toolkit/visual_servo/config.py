#!/usr/bin/env python3
"""
Visual Servoing Configuration Module
Provides configuration management for visual servoing parameters and safety limits
"""

import sys
from pathlib import Path

# Add setup directory to path for config manager
sys.path.insert(0, str(Path(__file__).parent.parent / "setup"))

from config_manager import config


class VisualServoConfig:
    """Configuration manager for visual servoing parameters"""

    def __init__(self):
        """Initialize with default configuration"""
        self.config = config

    @property
    def max_iterations(self) -> int:
        """Maximum number of correction iterations"""
        return self.config.get('visual_servo.max_iterations', 3)

    @property
    def detection_samples(self) -> int:
        """Number of detection samples for median filtering"""
        return self.config.get('visual_servo.detection_samples', 5)

    @property
    def position_tolerance(self) -> float:
        """Position tolerance for convergence (meters)"""
        return self.config.get('visual_servo.position_tolerance', 0.002)

    @property
    def rotation_tolerance(self) -> float:
        """Rotation tolerance for convergence (radians)"""
        return self.config.get('visual_servo.rotation_tolerance', 0.017)  # ~1 degree

    @property
    def max_translation_correction(self) -> float:
        """Maximum allowed translation correction per iteration (meters)"""
        return self.config.get('visual_servo.safety_limits.max_translation', 0.05)

    @property
    def max_rotation_correction(self) -> float:
        """Maximum allowed rotation correction per iteration (radians)"""
        return self.config.get('visual_servo.safety_limits.max_rotation', 0.35)  # ~20 degrees

    @property
    def max_total_correction(self) -> float:
        """Maximum total correction across all iterations (meters)"""
        return self.config.get('visual_servo.safety_limits.max_total_correction', 0.15)

    @property
    def damping_factor(self) -> float:
        """Damping factor for corrections to prevent oscillation"""
        return self.config.get('visual_servo.damping_factor', 0.7)

    @property
    def enable_pose_history(self) -> bool:
        """Whether to maintain pose correction history"""
        return self.config.get('visual_servo.enable_pose_history', True)

    @property
    def max_history_entries(self) -> int:
        """Maximum number of pose history entries to keep"""
        return self.config.get('visual_servo.max_history_entries', 50)

    @property
    def detection_timeout(self) -> float:
        """Timeout for AprilTag detection (seconds)"""
        return self.config.get('visual_servo.detection_timeout', 5.0)

    def print_config(self):
        """Print current visual servoing configuration"""
        print("🎯 Visual Servoing Configuration:")
        print(f"   Max iterations: {self.max_iterations}")
        print(f"   Detection samples: {self.detection_samples}")
        print(f"   Position tolerance: {self.position_tolerance:.4f}m")
        print(f"   Rotation tolerance: {self.rotation_tolerance:.4f}rad")
        print(f"   Max translation correction: {self.max_translation_correction:.3f}m")
        print(f"   Max rotation correction: {self.max_rotation_correction:.4f}rad")
        print(f"   Max total correction: {self.max_total_correction:.3f}m")
        print(f"   Damping factor: {self.damping_factor:.2f}")
        print(f"   Pose history enabled: {self.enable_pose_history}")
        print(f"   Detection timeout: {self.detection_timeout}s")

    def get_all_config(self):
        """Get all visual servo configuration as dictionary"""
        return {
            'max_iterations': self.max_iterations,
            'detection_samples': self.detection_samples,
            'position_tolerance': self.position_tolerance,
            'rotation_tolerance': self.rotation_tolerance,
            'max_translation_correction': self.max_translation_correction,
            'max_rotation_correction': self.max_rotation_correction,
            'max_total_correction': self.max_total_correction,
            'enable_pose_history': self.enable_pose_history,
            'max_history_entries': self.max_history_entries,
            'detection_timeout': self.detection_timeout
        }


# Global config instance
visual_servo_config = VisualServoConfig()
