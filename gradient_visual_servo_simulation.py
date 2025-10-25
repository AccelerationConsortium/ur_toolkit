"""
Simplified gradient-descent visual servoing simulation using moveL.

This simulation demonstrates a simple visual servoing approach that:
1. Uses only moveL commands (not speedL)
2. Approximates local gradient by testing small movements
3. Converges to target using gradient descent-like steps
4. Works with simulated UR robot kinematics
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from scipy.spatial.transform import Rotation as R
import urllib.request
import os

# Simulate UR robot kinematics (simplified)
class URKinematicsSimulator:
    """Simplified UR robot kinematics simulator."""
    
    def __init__(self):
        self.current_pose = np.array([0.3, -0.5, 0.3, 0.0, 0.0, 0.0])  # [x, y, z, rx, ry, rz]
        self.target_pose = np.array([0.0, -0.4, 0.4, 0.0, 0.0, 0.0])
        
    def moveL(self, target_pose, speed=0.05, accel=0.1):
        """Simulate moveL command - move to target pose."""
        # Simplified: just update current pose
        self.current_pose = np.array(target_pose)
        return True
        
    def get_current_pose(self):
        """Get current TCP pose."""
        return self.current_pose.copy()
        
    def compute_pose_error(self, current, target):
        """Compute pose error between current and target."""
        position_error = target[:3] - current[:3]
        rotation_error = target[3:] - current[3:]
        return np.concatenate([position_error, rotation_error])


class AprilTagCamera:
    """Simulated AprilTag camera for visual servoing."""
    
    def __init__(self, target_pose):
        self.target_pose = target_pose
        self.noise_std = 0.001  # 1mm position noise
        
    def detect_tag_relative_pose(self, robot_pose):
        """Detect AprilTag and return relative pose to target."""
        # Camera is at robot TCP
        # Return error from current to target
        error = self.target_pose - robot_pose
        
        # Add measurement noise
        noise = np.random.normal(0, self.noise_std, size=6)
        error[:3] += noise[:3]  # Position noise
        
        return error


class GradientVisualServo:
    """Gradient-descent visual servoing using moveL commands."""
    
    def __init__(self, robot, camera):
        self.robot = robot
        self.camera = camera
        self.step_sizes = np.array([0.01, 0.01, 0.01, 0.05, 0.05, 0.05])  # [x,y,z,rx,ry,rz]
        self.gradient_step = 0.002  # Small step for gradient approximation
        self.learning_rate = 0.5
        self.max_iterations = 100
        
    def approximate_gradient(self, current_pose):
        """
        Approximate gradient by testing small movements in each direction.
        This is the key insight: we don't need Jacobians, just test moves.
        """
        gradient = np.zeros(6)
        
        # Get current error
        current_error = self.camera.detect_tag_relative_pose(current_pose)
        current_cost = np.linalg.norm(current_error)
        
        # Test small movements in each direction
        for i in range(6):
            # Create test pose with small perturbation
            test_pose = current_pose.copy()
            test_pose[i] += self.gradient_step
            
            # Measure error at perturbed position
            test_error = self.camera.detect_tag_relative_pose(test_pose)
            test_cost = np.linalg.norm(test_error)
            
            # Approximate partial derivative
            gradient[i] = (test_cost - current_cost) / self.gradient_step
            
        return gradient
    
    def servo_to_target(self, tolerance=0.002):
        """
        Perform visual servoing to target using gradient descent.
        Returns: trajectory of poses
        """
        trajectory = []
        errors = []
        
        for iteration in range(self.max_iterations):
            # Get current pose
            current_pose = self.robot.get_current_pose()
            trajectory.append(current_pose.copy())
            
            # Get current error
            error = self.camera.detect_tag_relative_pose(current_pose)
            error_norm = np.linalg.norm(error[:3])  # Position error only
            errors.append(error_norm)
            
            print(f"Iteration {iteration}: Position error = {error_norm*1000:.1f}mm")
            
            # Check convergence
            if error_norm < tolerance:
                print(f"✅ Converged in {iteration} iterations")
                break
                
            # Approximate gradient
            gradient = self.approximate_gradient(current_pose)
            
            # Gradient descent step
            # Move in direction that reduces error
            step = -self.learning_rate * gradient * self.step_sizes
            
            # Compute new target pose
            new_pose = current_pose + step
            
            # Execute moveL command
            self.robot.moveL(new_pose)
            
        return np.array(trajectory), np.array(errors)


def generate_apriltag_pattern():
    """Generate a 6x6 AprilTag 36h11 pattern."""
    pattern = np.array([
        [1,1,1,1,1,1],
        [1,0,1,0,0,1],
        [1,1,1,1,0,1],
        [1,0,1,0,1,1],
        [1,1,0,1,1,1],
        [1,1,1,1,1,1]
    ], dtype=np.uint8)
    
    # Scale up for visibility
    tag_size = 60
    scaled = np.kron(pattern, np.ones((tag_size//6, tag_size//6), dtype=np.uint8))
    return scaled * 255


def create_visualization(trajectory, errors, output_file="gradient_servo_animation.gif"):
    """Create animated visualization of convergence."""
    
    # Generate AprilTag
    tag_pattern = generate_apriltag_pattern()
    
    fig = plt.figure(figsize=(14, 6))
    
    # 3D trajectory plot
    ax1 = fig.add_subplot(121, projection='3d')
    
    # Error plot
    ax2 = fig.add_subplot(122)
    
    # Initial setup
    target_pose = np.array([0.0, -0.4, 0.4, 0.0, 0.0, 0.0])
    
    def update(frame):
        ax1.clear()
        ax2.clear()
        
        # 3D trajectory
        ax1.plot(trajectory[:frame+1, 0], trajectory[:frame+1, 1], trajectory[:frame+1, 2], 
                'b-', linewidth=2, alpha=0.7, label='Path')
        ax1.scatter(trajectory[0, 0], trajectory[0, 1], trajectory[0, 2], 
                   c='g', s=200, marker='o', label='Start')
        ax1.scatter(target_pose[0], target_pose[1], target_pose[2], 
                   c='r', s=300, marker='*', label='Target')
        ax1.scatter(trajectory[frame, 0], trajectory[frame, 1], trajectory[frame, 2], 
                   c='b', s=150, marker='o', alpha=0.7)
        
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_zlabel('Z (m)')
        ax1.set_title(f'Iteration {frame}/{len(trajectory)-1}')
        ax1.legend()
        ax1.set_box_aspect([1,1,1])
        
        # Error convergence
        ax2.plot(range(frame+1), errors[:frame+1]*1000, 'b-', linewidth=2)
        ax2.axhline(y=2, color='r', linestyle='--', alpha=0.5, label='Target (2mm)')
        ax2.set_xlabel('Iteration')
        ax2.set_ylabel('Position Error (mm)')
        ax2.set_title('Convergence Progress')
        ax2.grid(True, alpha=0.3)
        ax2.legend()
        ax2.set_xlim(0, len(trajectory))
        ax2.set_ylim(0, max(errors)*1000*1.1)
        
        fig.suptitle(f'Gradient-Descent Visual Servoing (moveL)\nError: {errors[frame]*1000:.1f}mm', 
                     fontsize=14, fontweight='bold')
    
    # Create animation
    anim = FuncAnimation(fig, update, frames=len(trajectory), interval=100, repeat=True)
    
    # Save as GIF
    writer = PillowWriter(fps=10)
    anim.save(output_file, writer=writer)
    print(f"✅ Animation saved to {output_file}")
    
    plt.close()


def main():
    """Run gradient-descent visual servoing simulation."""
    print("=" * 60)
    print("Gradient-Descent Visual Servoing Simulation (moveL only)")
    print("=" * 60)
    
    # Initialize robot and camera
    robot = URKinematicsSimulator()
    target_pose = robot.target_pose
    camera = AprilTagCamera(target_pose)
    
    print(f"\n📍 Initial pose: {robot.current_pose[:3]*1000} mm")
    print(f"🎯 Target pose: {target_pose[:3]*1000} mm")
    print(f"📏 Initial error: {np.linalg.norm(robot.current_pose[:3] - target_pose[:3])*1000:.1f} mm\n")
    
    # Create visual servo controller
    servo = GradientVisualServo(robot, camera)
    
    # Run visual servoing
    print("🔄 Starting visual servoing with gradient approximation...\n")
    trajectory, errors = servo.servo_to_target()
    
    # Results
    final_error = errors[-1] * 1000
    print(f"\n📊 Final Results:")
    print(f"   Iterations: {len(trajectory)}")
    print(f"   Final position error: {final_error:.2f}mm")
    print(f"   Convergence: {'✅ Success' if final_error < 2 else '⚠️ Partial'}")
    
    # Create visualization
    print(f"\n🎨 Creating visualization...")
    create_visualization(trajectory, errors)
    
    print(f"\n✅ Simulation complete!")
    print(f"\nKey insight: No Jacobians needed! Just test small moves")
    print(f"to approximate gradient, then use moveL to converge.")


if __name__ == "__main__":
    main()
