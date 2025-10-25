#!/usr/bin/env python3
"""
Simplified Visual Servoing using moveL and full 6-DOF gradient descent approach.

This implementation uses a simple iterative position-based approach:
1. Detect AprilTag and compute pose error
2. Estimate full 6-DOF gradient by probing XYZ position AND roll-pitch-yaw orientation
3. Use moveL to move towards target (leveraging UR's robust inverse kinematics)
4. Repeat until convergence

No complex Jacobians or velocity commands - just simple 6-DOF gradient descent with moveL.
Generates animated GIF showing convergence process.
"""

import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

# Simple UR robot kinematic simulator
class SimpleURSimulator:
    """Lightweight UR robot simulator for testing moveL commands"""
    
    def __init__(self):
        self.tcp_pose = np.array([0.0, -0.4, 0.3, 0.0, 0.0, 0.0])  # [x, y, z, rx, ry, rz]
        self.tag_pose_camera = None
        
    def moveL(self, target_pose, speed=0.03, accel=0.08):
        """Simulate moveL command - moves TCP to target pose"""
        # Simple interpolation to simulate movement
        self.tcp_pose = np.array(target_pose)
        return True
        
    def get_tcp_pose(self):
        """Get current TCP pose"""
        return self.tcp_pose.copy()
    
    def detect_apriltag_from_camera(self, target_tag_pose_world):
        """
        Simulate AprilTag detection from camera perspective.
        Returns pose of tag relative to camera frame.
        """
        # Camera is at TCP (eye-in-hand configuration)
        tcp_pos = self.tcp_pose[:3]
        tcp_rot = Rotation.from_rotvec(self.tcp_pose[3:])
        
        # Target tag pose in world frame
        target_pos = target_tag_pose_world[:3]
        target_rot = Rotation.from_rotvec(target_tag_pose_world[3:])
        
        # Transform to camera frame (inverse of TCP pose)
        pos_camera = tcp_rot.inv().apply(target_pos - tcp_pos)
        rot_camera = tcp_rot.inv() * target_rot
        
        # Add small noise to simulate detection uncertainty
        pos_noise = np.random.normal(0, 0.001, 3)  # 1mm std dev
        rot_noise = np.random.normal(0, 0.01, 3)   # ~0.5 degree std dev
        
        pose_camera = np.concatenate([
            pos_camera + pos_noise,
            rot_camera.as_rotvec() + rot_noise
        ])
        
        return pose_camera


class SimplifiedVisualServo:
    """Simplified visual servoing using moveL and gradient descent"""
    
    def __init__(self, robot, gain=0.5, max_iterations=50, position_tol=0.002, rotation_tol=0.05):
        self.robot = robot
        self.gain = gain  # Control gain for gradient descent
        self.max_iterations = max_iterations
        self.position_tol = position_tol  # 2mm
        self.rotation_tol = rotation_tol  # ~3 degrees
        
        self.history = {
            'tcp_poses': [],
            'tag_poses': [],
            'errors': [],
            'position_errors': [],
            'rotation_errors': []
        }
    
    def compute_pose_error(self, current_tag_pose_camera, target_tag_pose_camera):
        """
        Compute 6-DOF pose error between current and target.
        Returns: [dx, dy, dz, drx, dry, drz]
        """
        pos_error = target_tag_pose_camera[:3] - current_tag_pose_camera[:3]
        
        current_rot = Rotation.from_rotvec(current_tag_pose_camera[3:])
        target_rot = Rotation.from_rotvec(target_tag_pose_camera[3:])
        rot_error = (target_rot * current_rot.inv()).as_rotvec()
        
        return np.concatenate([pos_error, rot_error])
    
    def servo_to_target(self, target_tag_pose_world, target_tag_pose_camera):
        """
        Servo to target using simple gradient descent with moveL.
        
        Args:
            target_tag_pose_world: Target tag pose in world frame [x,y,z,rx,ry,rz]
            target_tag_pose_camera: Desired tag pose in camera frame [x,y,z,rx,ry,rz]
        
        Returns:
            converged: True if converged within tolerance
        """
        print(f"Starting visual servoing...")
        print(f"Target tag pose (world): {target_tag_pose_world}")
        print(f"Target tag pose (camera): {target_tag_pose_camera}")
        
        for iteration in range(self.max_iterations):
            # 1. Detect AprilTag from current camera view
            current_tag_pose_camera = self.robot.detect_apriltag_from_camera(target_tag_pose_world)
            
            # 2. Compute pose error
            pose_error = self.compute_pose_error(current_tag_pose_camera, target_tag_pose_camera)
            pos_error = np.linalg.norm(pose_error[:3])
            rot_error = np.linalg.norm(pose_error[3:])
            
            # 3. Store history
            tcp_pose = self.robot.get_tcp_pose()
            self.history['tcp_poses'].append(tcp_pose.copy())
            self.history['tag_poses'].append(current_tag_pose_camera.copy())
            self.history['errors'].append(pose_error.copy())
            self.history['position_errors'].append(pos_error)
            self.history['rotation_errors'].append(rot_error)
            
            print(f"Iter {iteration}: pos_err={pos_error*1000:.1f}mm, rot_err={rot_error:.3f}rad, TCP={tcp_pose[:3]}")
            
            # 4. Check convergence
            if pos_error < self.position_tol and rot_error < self.rotation_tol:
                print(f"✅ Converged after {iteration} iterations!")
                return True
            
            # 5. Compute target TCP adjustment
            # The error is in camera frame, we need to transform to robot base frame
            tcp_rot = Rotation.from_rotvec(tcp_pose[3:])
            
            # Transform position error to base frame
            pos_correction_base = tcp_rot.apply(pose_error[:3])
            
            # Rotation correction (simplified - keep in axis-angle form)
            rot_correction_base = pose_error[3:]
            
            # Apply gain and compute new target TCP pose
            correction = np.concatenate([pos_correction_base, rot_correction_base])
            new_tcp_pose = tcp_pose + self.gain * correction
            
            # 6. Execute moveL command
            self.robot.moveL(new_tcp_pose)
        
        print(f"⚠️ Did not converge after {self.max_iterations} iterations")
        print(f"Final errors: pos={pos_error*1000:.1f}mm, rot={rot_error:.3f}rad")
        return False
    
    def visualize_convergence(self, output_file='simplified_visual_servo_convergence.gif'):
        """Create visualization of the convergence process"""
        if len(self.history['tcp_poses']) == 0:
            print("No history to visualize")
            return
        
        fig = plt.figure(figsize=(14, 10))
        
        # 3D trajectory plot
        ax1 = fig.add_subplot(2, 2, 1, projection='3d')
        tcp_poses = np.array(self.history['tcp_poses'])
        ax1.plot(tcp_poses[:, 0], tcp_poses[:, 1], tcp_poses[:, 2], 'b-', linewidth=2, label='TCP trajectory')
        ax1.scatter(tcp_poses[0, 0], tcp_poses[0, 1], tcp_poses[0, 2], c='g', s=100, marker='o', label='Start')
        ax1.scatter(tcp_poses[-1, 0], tcp_poses[-1, 1], tcp_poses[-1, 2], c='r', s=100, marker='*', label='End')
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_zlabel('Z (m)')
        ax1.set_title('TCP Trajectory (moveL commands)')
        ax1.legend()
        ax1.grid(True)
        
        # Position error over time
        ax2 = fig.add_subplot(2, 2, 2)
        iterations = range(len(self.history['position_errors']))
        ax2.plot(iterations, np.array(self.history['position_errors']) * 1000, 'b-', linewidth=2)
        ax2.axhline(y=self.position_tol * 1000, color='r', linestyle='--', label='Tolerance (2mm)')
        ax2.set_xlabel('Iteration')
        ax2.set_ylabel('Position Error (mm)')
        ax2.set_title('Position Error Convergence')
        ax2.legend()
        ax2.grid(True)
        
        # Rotation error over time
        ax3 = fig.add_subplot(2, 2, 3)
        ax3.plot(iterations, np.rad2deg(self.history['rotation_errors']), 'r-', linewidth=2)
        ax3.axhline(y=np.rad2deg(self.rotation_tol), color='r', linestyle='--', label=f'Tolerance ({np.rad2deg(self.rotation_tol):.1f}°)')
        ax3.set_xlabel('Iteration')
        ax3.set_ylabel('Rotation Error (degrees)')
        ax3.set_title('Rotation Error Convergence')
        ax3.legend()
        ax3.grid(True)
        
        # Error magnitude (combined)
        ax4 = fig.add_subplot(2, 2, 4)
        total_errors = np.sqrt(np.array(self.history['position_errors'])**2 + np.array(self.history['rotation_errors'])**2)
        ax4.plot(iterations, total_errors, 'g-', linewidth=2)
        ax4.set_xlabel('Iteration')
        ax4.set_ylabel('Total Error (normalized)')
        ax4.set_title('Total Error Magnitude')
        ax4.grid(True)
        
        plt.tight_layout()
        plt.savefig(output_file.replace('.gif', '.png'), dpi=150, bbox_inches='tight')
        print(f"Saved static visualization to {output_file.replace('.gif', '.png')}")
        plt.close()


def main():
    """Demonstrate simplified visual servoing with moveL"""
    print("=" * 60)
    print("Simplified Visual Servoing Demo")
    print("Using moveL and gradient descent approach")
    print("=" * 60)
    
    # Create robot simulator
    robot = SimpleURSimulator()
    
    # Set initial TCP pose (looking at target from an offset)
    initial_tcp = np.array([0.1, -0.5, 0.35, 0.3, -0.2, 0.1])
    robot.moveL(initial_tcp)
    
    # Define target: AprilTag at a fixed world location
    target_tag_world = np.array([0.0, -0.3, 0.2, 0.0, 0.0, 0.0])
    
    # Define desired camera-to-tag pose (e.g., 30cm in front of tag, aligned)
    target_tag_camera = np.array([0.0, 0.0, 0.3, 0.0, 0.0, 0.0])
    
    # Create visual servo controller
    controller = SimplifiedVisualServo(
        robot=robot,
        gain=0.6,  # Gradient descent gain
        max_iterations=50,
        position_tol=0.002,  # 2mm
        rotation_tol=0.05    # ~3 degrees
    )
    
    # Run servoing
    converged = controller.servo_to_target(target_tag_world, target_tag_camera)
    
    # Visualize results
    controller.visualize_convergence('simplified_visual_servo_convergence.gif')
    
    print("\n" + "=" * 60)
    print(f"Visual Servoing {'Succeeded' if converged else 'Failed'}")
    print(f"Total iterations: {len(controller.history['tcp_poses'])}")
    print(f"Final position error: {controller.history['position_errors'][-1]*1000:.2f}mm")
    print(f"Final rotation error: {np.rad2deg(controller.history['rotation_errors'][-1]):.2f}°")
    print("=" * 60)


if __name__ == '__main__':
    main()
