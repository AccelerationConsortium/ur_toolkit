"""
Simplified visual servoing simulation using moveL and gradient-based convergence.

This simulation demonstrates a simplified approach to visual servoing:
1. Uses moveL for all movements (coordinates-based, leveraging UR's inverse kinematics)
2. Simple gradient-descent approach: make small movements and observe error reduction
3. Uses roboticstoolbox-python to simulate UR5 robot kinematics
4. Simulates AprilTag detection in camera frame
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation
import roboticstoolbox as rtb
from spatialmath import SE3

# Simulation parameters
MAX_ITERATIONS = 50
STEP_SIZE = 0.01  # meters
ANGLE_STEP = 0.05  # radians
CONVERGENCE_THRESHOLD_POS = 0.005  # 5mm
CONVERGENCE_THRESHOLD_ROT = 0.05  # ~3 degrees


class SimplifiedVisualServoSimulator:
    """Simplified visual servoing using moveL and gradient descent."""
    
    def __init__(self):
        # Initialize UR5 robot model
        self.robot = rtb.models.DH.UR5()
        
        # Camera parameters (eye-in-hand configuration)
        self.camera_offset = SE3(0, 0, 0.1)  # Camera 10cm from TCP
        
        # Initial and target poses
        self.initial_pose = SE3(0.3, -0.2, 0.4) * SE3.RPY([0.5, -0.3, 0.2])
        self.target_pose = SE3(0.4, 0.1, 0.35) * SE3.RPY([0, 0, 0])
        
        # AprilTag pose in world frame (fixed)
        self.tag_world_pose = self.target_pose
        
        # Current TCP pose
        self.current_pose = self.initial_pose.copy()
        
        # History for visualization
        self.pose_history = [self.current_pose.copy()]
        self.error_history = []
        
    def get_camera_pose(self):
        """Get current camera pose in world frame."""
        return self.current_pose * self.camera_offset
    
    def detect_apriltag(self):
        """
        Simulate AprilTag detection.
        Returns tag pose in camera frame and error metrics.
        """
        camera_pose = self.get_camera_pose()
        
        # Transform tag from world to camera frame
        tag_in_camera = camera_pose.inv() * self.tag_world_pose
        
        # Extract position and orientation error
        position_error = tag_in_camera.t
        rotation_matrix = tag_in_camera.R
        rotation = Rotation.from_matrix(rotation_matrix)
        rotation_error = rotation.as_rotvec()
        
        return position_error, rotation_error
    
    def compute_error_magnitude(self):
        """Compute total error magnitude."""
        pos_err, rot_err = self.detect_apriltag()
        pos_magnitude = np.linalg.norm(pos_err)
        rot_magnitude = np.linalg.norm(rot_err)
        return pos_magnitude, rot_magnitude
    
    def try_movement(self, delta_pose):
        """
        Simulate moveL command with small delta.
        Returns error after movement.
        """
        test_pose = self.current_pose * delta_pose
        
        # Simulate movement
        old_pose = self.current_pose
        self.current_pose = test_pose
        
        # Measure error
        pos_err_mag, rot_err_mag = self.compute_error_magnitude()
        
        # Restore pose
        self.current_pose = old_pose
        
        return pos_err_mag + rot_err_mag
    
    def gradient_descent_step(self):
        """
        Perform one iteration of gradient-based visual servoing.
        Try small movements in different directions and pick the best.
        """
        current_error = sum(self.compute_error_magnitude())
        
        # Define test movements (6 DOF: +/- x, y, z, rx, ry, rz)
        test_deltas = [
            SE3(STEP_SIZE, 0, 0),   # +x
            SE3(-STEP_SIZE, 0, 0),  # -x
            SE3(0, STEP_SIZE, 0),   # +y
            SE3(0, -STEP_SIZE, 0),  # -y
            SE3(0, 0, STEP_SIZE),   # +z
            SE3(0, 0, -STEP_SIZE),  # -z
            SE3.RPY([ANGLE_STEP, 0, 0]),   # +rx
            SE3.RPY([-ANGLE_STEP, 0, 0]),  # -rx
            SE3.RPY([0, ANGLE_STEP, 0]),   # +ry
            SE3.RPY([0, -ANGLE_STEP, 0]),  # -ry
            SE3.RPY([0, 0, ANGLE_STEP]),   # +rz
            SE3.RPY([0, 0, -ANGLE_STEP]),  # -rz
        ]
        
        # Test each direction
        best_error = current_error
        best_delta = None
        
        for delta in test_deltas:
            test_error = self.try_movement(delta)
            if test_error < best_error:
                best_error = test_error
                best_delta = delta
        
        # Apply best movement (moveL command)
        if best_delta is not None:
            self.current_pose = self.current_pose * best_delta
            self.pose_history.append(self.current_pose.copy())
            self.error_history.append(best_error)
            return True
        else:
            # No improvement found, converged
            self.error_history.append(current_error)
            return False
    
    def run_simulation(self):
        """Run the complete visual servoing simulation."""
        print("Starting simplified visual servoing simulation...")
        print(f"Initial error: {sum(self.compute_error_magnitude()):.4f}")
        
        for iteration in range(MAX_ITERATIONS):
            pos_err, rot_err = self.compute_error_magnitude()
            
            # Check convergence
            if pos_err < CONVERGENCE_THRESHOLD_POS and rot_err < CONVERGENCE_THRESHOLD_ROT:
                print(f"\nConverged at iteration {iteration}!")
                print(f"Final position error: {pos_err*1000:.2f}mm")
                print(f"Final rotation error: {rot_err:.3f}rad ({np.degrees(rot_err):.1f}°)")
                break
            
            # Perform gradient descent step
            improved = self.gradient_descent_step()
            
            if iteration % 5 == 0:
                print(f"Iteration {iteration}: pos_err={pos_err*1000:.1f}mm, rot_err={np.degrees(rot_err):.1f}°")
            
            if not improved and iteration > 5:
                print(f"\nNo improvement found at iteration {iteration}. Stopping.")
                break
        
        print(f"\nSimulation complete. Total iterations: {len(self.error_history)}")
        return self.pose_history, self.error_history


def visualize_results(pose_history, error_history):
    """Create visualization of the visual servoing process."""
    fig = plt.figure(figsize=(15, 5))
    
    # Extract positions
    positions = np.array([pose.t for pose in pose_history])
    
    # 1. 3D trajectory
    ax1 = fig.add_subplot(131, projection='3d')
    ax1.plot(positions[:, 0], positions[:, 1], positions[:, 2], 
             'b-', linewidth=2, label='TCP trajectory')
    ax1.scatter(positions[0, 0], positions[0, 1], positions[0, 2], 
                c='green', s=100, marker='o', label='Start')
    ax1.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2], 
                c='red', s=100, marker='*', label='End')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('TCP Trajectory')
    ax1.legend()
    ax1.grid(True)
    
    # 2. Error convergence
    ax2 = fig.add_subplot(132)
    iterations = range(len(error_history))
    ax2.plot(iterations, error_history, 'b-', linewidth=2)
    ax2.set_xlabel('Iteration')
    ax2.set_ylabel('Total Error')
    ax2.set_title('Error Convergence')
    ax2.grid(True)
    ax2.set_yscale('log')
    
    # 3. Position over time
    ax3 = fig.add_subplot(133)
    ax3.plot(positions[:, 0], label='X')
    ax3.plot(positions[:, 1], label='Y')
    ax3.plot(positions[:, 2], label='Z')
    ax3.set_xlabel('Iteration')
    ax3.set_ylabel('Position (m)')
    ax3.set_title('TCP Position Components')
    ax3.legend()
    ax3.grid(True)
    
    plt.tight_layout()
    plt.savefig('simplified_visual_servo_results.png', dpi=150, bbox_inches='tight')
    print("Saved visualization to: simplified_visual_servo_results.png")
    plt.close()


def create_animation(pose_history, error_history):
    """Create animated GIF of the convergence process."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    
    positions = np.array([pose.t for pose in pose_history])
    
    def update(frame):
        ax1.clear()
        ax2.clear()
        
        # 3D trajectory up to current frame
        ax1.plot(positions[:frame+1, 0], positions[:frame+1, 1], 
                positions[:frame+1, 2], 'b-', linewidth=2)
        ax1.scatter(positions[0, 0], positions[0, 1], positions[0, 2], 
                   c='green', s=100, marker='o', label='Start')
        ax1.scatter(positions[frame, 0], positions[frame, 1], positions[frame, 2], 
                   c='blue', s=100, marker='o', label='Current')
        ax1.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2], 
                   c='red', s=100, marker='*', label='Target')
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_title(f'Iteration {frame}/{len(pose_history)-1}')
        ax1.legend()
        ax1.grid(True)
        
        # Set consistent axis limits
        ax1.set_xlim(positions[:, 0].min()-0.05, positions[:, 0].max()+0.05)
        ax1.set_ylim(positions[:, 1].min()-0.05, positions[:, 1].max()+0.05)
        
        # Error history
        ax2.plot(range(frame+1), error_history[:frame+1], 'b-', linewidth=2)
        ax2.set_xlabel('Iteration')
        ax2.set_ylabel('Total Error')
        ax2.set_title('Error Convergence')
        ax2.grid(True)
        ax2.set_xlim(0, len(error_history))
        ax2.set_ylim(0, max(error_history)*1.1)
        ax2.set_yscale('log')
    
    # Create animation with fewer frames for smaller file size
    frame_indices = list(range(0, len(pose_history), max(1, len(pose_history)//30)))
    if frame_indices[-1] != len(pose_history)-1:
        frame_indices.append(len(pose_history)-1)
    
    anim = FuncAnimation(fig, update, frames=frame_indices, 
                        interval=200, repeat=True)
    anim.save('simplified_visual_servo_animation.gif', writer='pillow', fps=5)
    print("Saved animation to: simplified_visual_servo_animation.gif")
    plt.close()


if __name__ == "__main__":
    # Run simulation
    simulator = SimplifiedVisualServoSimulator()
    pose_history, error_history = simulator.run_simulation()
    
    # Visualize results
    visualize_results(pose_history, error_history)
    create_animation(pose_history, error_history)
    
    print("\nSimulation complete!")
    print(f"Files generated:")
    print("  - simplified_visual_servo_results.png")
    print("  - simplified_visual_servo_animation.gif")
