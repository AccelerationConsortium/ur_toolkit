#!/usr/bin/env python3
"""
6-DOF Visual Servoing using moveL with Direct Error Correction.

This implementation uses the detected AprilTag pose error directly:
1. Detect AprilTag and compute full 6-DOF pose error vector [x, y, z, roll, pitch, yaw]
2. Apply ONE moveL command with scaled error as correction (corrects all 6 DOF simultaneously)
3. Use momentum damping for smooth convergence
4. Repeat until convergence

Key insight: The AprilTag detector already gives us the full 6-DOF error. We don't need
to probe in multiple directions - just directly use the error vector and move to reduce it.
All 6 dimensions are corrected together in each moveL command for smooth, direct convergence.

Uses UR robot's robust inverse kinematics with simple moveL commands.
No complex Jacobians, velocity control, or gradient probing required.
"""

import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
from matplotlib.animation import PillowWriter

class SimpleURSimulator:
    """Lightweight UR robot simulator for testing moveL commands"""
    
    def __init__(self):
        self.tcp_pose = np.array([0.0, -0.4, 0.3, 0.1, 0.2, -0.15])  # [x, y, z, rx, ry, rz] in radians
        
    def moveL(self, target_pose):
        """Simulate moveL command - moves TCP to target pose"""
        self.tcp_pose = np.array(target_pose)
        return True
        
    def get_tcp_pose(self):
        """Get current TCP pose [x, y, z, rx, ry, rz]"""
        return self.tcp_pose.copy()
    
    def detect_apriltag(self, target_pose_world):
        """
        Simulate AprilTag detection from camera.
        Returns pose error between current and target.
        """
        # Compute 6-DOF error
        pos_error = target_pose_world[:3] - self.tcp_pose[:3]
        
        # Rotation error using axis-angle representation
        current_rot = Rotation.from_rotvec(self.tcp_pose[3:])
        target_rot = Rotation.from_rotvec(target_pose_world[3:])
        rot_error_matrix = target_rot * current_rot.inv()
        rot_error = rot_error_matrix.as_rotvec()
        
        return np.concatenate([pos_error, rot_error])


def visual_servo_gradient_descent():
    """
    Run visual servoing with 6-DOF gradient descent using moveL.
    
    At each iteration:
    1. Detect AprilTag and get 6-DOF pose error (all components at once)
    2. Apply one moveL command with a scaled version of the error as correction
    
    This corrects position and orientation simultaneously in each move based on
    the detected AprilTag pose error.
    """
    # Initialize robot simulator
    robot = SimpleURSimulator()
    
    # Target pose (where we want the robot to go)
    target_pose = np.array([0.1, -0.3, 0.35, 0.0, 0.0, 0.0])
    
    # Control parameters
    max_iterations = 50
    first_move_gain = 0.97  # Aggressive first move to get very close (97% of error)
    subsequent_gain = 0.2  # Conservative for fine-tuning (20% of remaining error)
    momentum = 0.5  # Smoothing factor to reduce oscillation
    convergence_threshold = 0.001  # 1mm position error
    
    # Storage for visualization and momentum
    poses_history = []
    errors_history = []
    velocity = np.zeros(6)  # Accumulated velocity for momentum
    
    print("Starting 6-DOF Visual Servoing with Combined Error Correction")
    print(f"Initial pose: {robot.get_tcp_pose()}")
    print(f"Target pose:  {target_pose}")
    
    current_pose = robot.get_tcp_pose()
    error_vec = robot.detect_apriltag(target_pose)
    initial_error = np.linalg.norm(error_vec)
    print(f"Initial error: {initial_error:.4f} m\n")
    
    for iteration in range(max_iterations):
        # Get current pose and detect full 6-DOF error vector
        current_pose = robot.get_tcp_pose()
        error_vec = robot.detect_apriltag(target_pose)
        current_error = np.linalg.norm(error_vec)
        
        # Store for visualization
        poses_history.append(current_pose.copy())
        errors_history.append(current_error)
        
        print(f"Iteration {iteration:2d}: Error = {current_error:.6f} m, "
              f"Error vector = [{error_vec[0]:.3f}, {error_vec[1]:.3f}, {error_vec[2]:.3f}, "
              f"{error_vec[3]:.3f}, {error_vec[4]:.3f}, {error_vec[5]:.3f}]")
        
        # Check convergence
        if current_error < convergence_threshold:
            print(f"\n✓ Converged after {iteration} iterations!")
            break
        
        # The correction is proportional to the error vector
        # The error vector points FROM current TO target, so we move IN that direction
        correction = error_vec  # Positive - move in direction of error to reduce it
        
        # Select gain based on iteration (first move is aggressive, rest are conservative)
        current_gain = first_move_gain if iteration == 0 else subsequent_gain
        
        # Apply momentum for smoother convergence
        velocity = momentum * velocity + (1 - momentum) * correction
        
        # Compute new pose by applying combined correction to ALL 6 DOF at once
        new_pose = current_pose + current_gain * velocity
        
        # Execute single moveL command with combined 6-DOF correction
        robot.moveL(new_pose)
    
    # Final measurements
    final_pose = robot.get_tcp_pose()
    final_error_vec = robot.detect_apriltag(target_pose)
    final_error = np.linalg.norm(final_error_vec)
    
    poses_history.append(final_pose.copy())
    errors_history.append(final_error)
    
    print(f"\nFinal pose:  {final_pose}")
    print(f"Final error: {final_error:.6f} m")
    
    return np.array(poses_history), np.array(errors_history), target_pose


def create_visualization(poses_history, errors_history, target_pose):
    """Create animated visualization of convergence process"""
    
    fig = plt.figure(figsize=(14, 5))
    
    # 3D trajectory plot
    ax1 = fig.add_subplot(131, projection='3d')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('6-DOF Visual Servoing Trajectory\n(moveL + Combined Gradient Descent)')
    
    # Plot full trajectory
    ax1.plot(poses_history[:, 0], poses_history[:, 1], poses_history[:, 2], 
             'b-', alpha=0.3, linewidth=2, label='Path')
    ax1.scatter(poses_history[0, 0], poses_history[0, 1], poses_history[0, 2],
                c='green', s=100, marker='o', label='Start')
    ax1.scatter(target_pose[0], target_pose[1], target_pose[2],
                c='red', s=100, marker='*', label='Target')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Error convergence plot
    ax2 = fig.add_subplot(132)
    ax2.plot(errors_history, 'b-', linewidth=2)
    ax2.set_xlabel('Iteration')
    ax2.set_ylabel('Error (m)')
    ax2.set_title('Convergence of 6-DOF Error')
    ax2.grid(True, alpha=0.3)
    ax2.set_yscale('log')
    
    # Pose components over time
    ax3 = fig.add_subplot(133)
    iterations = np.arange(len(poses_history))
    ax3.plot(iterations, poses_history[:, 0], 'r-', label='X', alpha=0.7)
    ax3.plot(iterations, poses_history[:, 1], 'g-', label='Y', alpha=0.7)
    ax3.plot(iterations, poses_history[:, 2], 'b-', label='Z', alpha=0.7)
    ax3.axhline(target_pose[0], color='r', linestyle='--', alpha=0.3)
    ax3.axhline(target_pose[1], color='g', linestyle='--', alpha=0.3)
    ax3.axhline(target_pose[2], color='b', linestyle='--', alpha=0.3)
    ax3.set_xlabel('Iteration')
    ax3.set_ylabel('Position (m)')
    ax3.set_title('Position Convergence (XYZ)')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    return fig


def create_animation_gif(poses_history, errors_history, target_pose, filename='moveL_6dof_convergence.gif'):
    """Create animated GIF showing convergence process"""
    
    fig = plt.figure(figsize=(12, 4))
    
    # 3D trajectory
    ax1 = fig.add_subplot(131, projection='3d')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('Trajectory')
    
    # Error plot
    ax2 = fig.add_subplot(132)
    ax2.set_xlabel('Iteration')
    ax2.set_ylabel('Error (m)')
    ax2.set_title('Error Convergence')
    ax2.set_yscale('log')
    ax2.grid(True, alpha=0.3)
    
    # RPY plot
    ax3 = fig.add_subplot(133)
    ax3.set_xlabel('Iteration')
    ax3.set_ylabel('Angle (rad)')
    ax3.set_title('Orientation (RPY)')
    ax3.grid(True, alpha=0.3)
    
    # Plot target
    ax1.scatter(target_pose[0], target_pose[1], target_pose[2],
                c='red', s=100, marker='*', label='Target', zorder=100)
    
    def update(frame):
        # Clear previous frame data
        for ax in [ax1, ax2, ax3]:
            for artist in ax.lines + ax.collections:
                if hasattr(artist, 'remove'):
                    artist.remove()
        
        # Replot target
        ax1.scatter(target_pose[0], target_pose[1], target_pose[2],
                    c='red', s=100, marker='*', label='Target', zorder=100)
        
        # Plot trajectory up to current frame
        ax1.plot(poses_history[:frame+1, 0], poses_history[:frame+1, 1], 
                poses_history[:frame+1, 2], 'b-', alpha=0.5, linewidth=2)
        ax1.scatter(poses_history[frame, 0], poses_history[frame, 1], 
                   poses_history[frame, 2], c='blue', s=80, marker='o', zorder=50)
        ax1.scatter(poses_history[0, 0], poses_history[0, 1], poses_history[0, 2],
                   c='green', s=80, marker='o', label='Start')
        
        # Error convergence
        ax2.plot(errors_history[:frame+1], 'b-', linewidth=2)
        ax2.scatter(frame, errors_history[frame], c='blue', s=50, zorder=50)
        
        # Orientation convergence
        iterations = np.arange(frame+1)
        ax3.plot(iterations, poses_history[:frame+1, 3], 'r-', label='Roll', alpha=0.7)
        ax3.plot(iterations, poses_history[:frame+1, 4], 'g-', label='Pitch', alpha=0.7)
        ax3.plot(iterations, poses_history[:frame+1, 5], 'b-', label='Yaw', alpha=0.7)
        if frame == 0:
            ax3.legend()
        
        # Update title with current error
        fig.suptitle(f'6-DOF Visual Servoing (moveL + Direct Error Correction)\n'
                    f'Iteration {frame}/{len(poses_history)-1} | '
                    f'Error: {errors_history[frame]:.6f} m',
                    fontsize=12)
        
        return []
    
    # Create animation
    frames = len(poses_history)
    # Sample frames for reasonable file size
    frame_indices = list(range(0, frames, max(1, frames//30))) + [frames-1]
    
    from matplotlib.animation import FuncAnimation
    anim = FuncAnimation(fig, update, frames=frame_indices, 
                        interval=200, blit=False, repeat=True)
    
    # Save as GIF
    writer = PillowWriter(fps=5)
    anim.save(filename, writer=writer)
    print(f"\n✓ Animation saved to {filename}")
    
    plt.close()


if __name__ == "__main__":
    # Run simulation
    poses_history, errors_history, target_pose = visual_servo_gradient_descent()
    
    # Create visualization
    fig = create_visualization(poses_history, errors_history, target_pose)
    plt.savefig('moveL_6dof_convergence.png', dpi=150, bbox_inches='tight')
    print("✓ Static visualization saved to moveL_6dof_convergence.png")
    
    # Create animated GIF
    create_animation_gif(poses_history, errors_history, target_pose)
    
    plt.show()
