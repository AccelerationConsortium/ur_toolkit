#!/usr/bin/env python3
"""
Kalman Filter for Pose Smoothing
Filters pose measurements to reduce noise and improve convergence
"""

import cv2
import numpy as np
from typing import Optional


class PoseKalmanFilter:
    """Kalman filter for 6DOF pose smoothing"""
    
    def __init__(self, dt: float = 0.1, process_noise: float = 0.01, measurement_noise: float = 0.1):
        """
        Initialize Kalman filter for pose tracking
        
        Args:
            dt: Time step (seconds)
            process_noise: Process noise covariance
            measurement_noise: Measurement noise covariance
        """
        self.dt = dt
        self.initialized = False
        
        # State: [x, y, z, rx, ry, rz, vx, vy, vz, vrx, vry, vrz]
        # 12 states (pose + velocity)
        self.kf = cv2.KalmanFilter(12, 6)
        
        # State transition model (constant velocity)
        self.kf.transitionMatrix = np.eye(12, dtype=np.float32)
        for i in range(6):
            self.kf.transitionMatrix[i, i+6] = dt  # position += velocity * dt
            
        # Measurement model (observe position only)
        self.kf.measurementMatrix = np.zeros((6, 12), dtype=np.float32)
        for i in range(6):
            self.kf.measurementMatrix[i, i] = 1.0
            
        # Process noise covariance
        self.kf.processNoiseCov = np.eye(12, dtype=np.float32) * process_noise
        
        # Measurement noise covariance
        self.kf.measurementNoiseCov = np.eye(6, dtype=np.float32) * measurement_noise
        
        # Error covariance
        self.kf.errorCovPost = np.eye(12, dtype=np.float32) * 0.1
        
        print(f"🔧 Kalman filter initialized (dt={dt}, process_noise={process_noise}, measurement_noise={measurement_noise})")
    
    def update(self, measurement: np.ndarray, dt: Optional[float] = None) -> np.ndarray:
        """
        Update filter with new pose measurement
        
        Args:
            measurement: 6DOF pose measurement [x, y, z, rx, ry, rz]
            dt: Time step (optional, uses default if None)
            
        Returns:
            Filtered 6DOF pose
        """
        if dt is not None and dt != self.dt:
            self._update_transition_matrix(dt)
            
        measurement = measurement.astype(np.float32)
        
        if not self.initialized:
            # Initialize state with first measurement
            initial_state = np.zeros(12, dtype=np.float32)
            initial_state[:6] = measurement  # Set initial pose
            initial_state[6:] = 0.0          # Zero initial velocity
            
            self.kf.statePre = initial_state.copy()
            self.kf.statePost = initial_state.copy()
            
            self.initialized = True
            print("✅ Kalman filter initialized with first measurement")
            return measurement
        
        # Prediction step
        prediction = self.kf.predict()
        
        # Correction step
        corrected = self.kf.correct(measurement)
        
        # Return filtered pose (first 6 states)
        return corrected[:6]
    
    def _update_transition_matrix(self, dt: float):
        """Update transition matrix with new time step"""
        self.dt = dt
        for i in range(6):
            self.kf.transitionMatrix[i, i+6] = dt
    
    def get_velocity(self) -> Optional[np.ndarray]:
        """
        Get estimated velocity from filter
        
        Returns:
            6DOF velocity [vx, vy, vz, vrx, vry, vrz] or None if not initialized
        """
        if not self.initialized:
            return None
            
        return self.kf.statePost[6:12].copy()
    
    def reset(self):
        """Reset filter state"""
        self.initialized = False
        print("🔄 Kalman filter reset")
    
    def is_stable(self, velocity_threshold: float = 0.001) -> bool:
        """
        Check if pose is stable (low velocity)
        
        Args:
            velocity_threshold: Maximum velocity for stability
            
        Returns:
            True if pose is stable
        """
        if not self.initialized:
            return False
            
        velocity = self.get_velocity()
        if velocity is None:
            return False
            
        velocity_magnitude = np.linalg.norm(velocity)
        return velocity_magnitude < velocity_threshold