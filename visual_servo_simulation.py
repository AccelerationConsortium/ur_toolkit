#!/usr/bin/env python3
"""
Visual Servoing Simulation with speedL Control
Demonstrates PBVS approach using Cartesian velocity control

This simulation demonstrates the speedL() implementation recommended for
visual servoing with UR robots. It shows how Cartesian velocity control
can achieve smooth convergence to a target pose.

Usage:
    python3 visual_servo_simulation.py

Output:
    - visual_servo_trajectory.png: Full trajectory with error/velocity plots
    - visual_servo_snapshots.png: Snapshots at different stages

The simulation models:
    - PBVS control law with gain λ
    - speedL() Cartesian velocity commands
    - Measurement noise (realistic sensor noise)
    - 30 Hz control rate (typical for vision feedback)
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d
from scipy.spatial.transform import Rotation as R
import time

class Arrow3D(FancyArrowPatch):
    """3D arrow for visualization"""
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0, 0), (0, 0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def do_3d_projection(self, renderer=None):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        return np.min(zs)

class VisualServoSimulation:
    """Simulates PBVS with speedL control"""
    
    def __init__(self):
        # Target AprilTag pose (goal)
        self.target_pose = np.array([0.5, 0.3, 0.4, 0.0, 0.0, 0.0])
        
        # Initial robot pose (offset from target with significant rotation)
        # Large rotation offsets make orientation convergence more visible
        self.current_pose = np.array([0.4, 0.25, 0.35, 0.5, -0.4, 0.3])
        
        # Control parameters
        self.control_gain = 0.5  # Increased for faster convergence
        self.dt = 1.0/30.0  # 30 Hz
        self.max_velocity = 0.1  # m/s (increased)
        
        # History tracking
        self.pose_history = [self.current_pose.copy()]
        self.error_history = []
        self.velocity_history = []
        
        # Noise simulation
        self.measurement_noise_std = 0.001  # 1mm std dev
        
    def add_measurement_noise(self, pose):
        """Add realistic measurement noise"""
        noise = np.random.normal(0, self.measurement_noise_std, 6)
        noise[3:] *= 0.01  # Less noise on orientation
        return pose + noise
    
    def compute_pose_error(self, current, target):
        """Compute SE(3) pose error"""
        # Simple difference for simulation
        error = target - current
        return error
    
    def speedL_control(self, pose_error):
        """Simulate speedL Cartesian velocity control"""
        # Compute velocity command
        v_task = self.control_gain * pose_error
        
        # Apply velocity limits
        trans_vel = np.linalg.norm(v_task[:3])
        if trans_vel > self.max_velocity:
            v_task[:3] *= self.max_velocity / trans_vel
        
        return v_task
    
    def step(self):
        """Execute one control iteration"""
        # Measure current pose (with noise)
        measured_pose = self.add_measurement_noise(self.current_pose)
        
        # Compute error
        pose_error = self.compute_pose_error(measured_pose, self.target_pose)
        self.error_history.append(np.linalg.norm(pose_error[:3]))
        
        # Compute velocity command using speedL
        v_task = self.speedL_control(pose_error)
        self.velocity_history.append(np.linalg.norm(v_task[:3]))
        
        # Integrate velocity to update pose
        self.current_pose += v_task * self.dt
        self.pose_history.append(self.current_pose.copy())
        
        return np.linalg.norm(pose_error[:3])
    
    def run_simulation(self, max_iters=50, tolerance=0.002):
        """Run visual servoing simulation"""
        print("🎯 Visual Servoing Simulation Starting")
        print(f"Initial error: {np.linalg.norm(self.current_pose[:3] - self.target_pose[:3]):.4f}m")
        
        for i in range(max_iters):
            error = self.step()
            
            if i % 10 == 0:
                print(f"Iter {i:2d}: Error = {error:.4f}m, Velocity = {self.velocity_history[-1]:.4f}m/s")
            
            if error < tolerance:
                print(f"✅ Converged at iteration {i+1}")
                print(f"Final error: {error:.4f}m")
                return True
        
        print(f"⚠️  Did not converge within {max_iters} iterations")
        print(f"Final error: {error:.4f}m")
        return False
    
    def visualize_trajectory(self, filename='visual_servo_trajectory.png'):
        """Create 3D trajectory visualization"""
        fig = plt.figure(figsize=(15, 10))
        
        # 3D trajectory plot
        ax1 = fig.add_subplot(2, 2, 1, projection='3d')
        poses = np.array(self.pose_history)
        
        # Plot trajectory
        ax1.plot(poses[:, 0], poses[:, 1], poses[:, 2], 
                'b-', linewidth=2, label='Robot trajectory')
        ax1.scatter(poses[0, 0], poses[0, 1], poses[0, 2], 
                   c='green', s=100, marker='o', label='Start')
        ax1.scatter(poses[-1, 0], poses[-1, 1], poses[-1, 2], 
                   c='blue', s=100, marker='o', label='End')
        ax1.scatter(self.target_pose[0], self.target_pose[1], self.target_pose[2],
                   c='red', s=200, marker='*', label='Target')
        
        # Draw coordinate frames at start and end
        scale = 0.05
        # Start frame (green)
        start_pos = poses[0, :3]
        for i, color in enumerate(['red', 'green', 'blue']):
            vec = np.zeros(3)
            vec[i] = scale
            arrow = Arrow3D([start_pos[0], start_pos[0]+vec[0]],
                          [start_pos[1], start_pos[1]+vec[1]],
                          [start_pos[2], start_pos[2]+vec[2]],
                          mutation_scale=10, lw=2, arrowstyle='->', color=color, alpha=0.5)
            ax1.add_artist(arrow)
        
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_zlabel('Z (m)')
        ax1.set_title('3D Trajectory (speedL Control)', fontsize=12, fontweight='bold')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # Error over time
        ax2 = fig.add_subplot(2, 2, 2)
        iterations = np.arange(len(self.error_history))
        ax2.plot(iterations, np.array(self.error_history) * 1000, 'r-', linewidth=2)
        ax2.axhline(y=2, color='g', linestyle='--', label='Tolerance (2mm)')
        ax2.set_xlabel('Iteration')
        ax2.set_ylabel('Position Error (mm)')
        ax2.set_title('Convergence Plot', fontsize=12, fontweight='bold')
        ax2.grid(True, alpha=0.3)
        ax2.legend()
        
        # Velocity over time
        ax3 = fig.add_subplot(2, 2, 3)
        ax3.plot(iterations, np.array(self.velocity_history) * 1000, 'b-', linewidth=2)
        ax3.set_xlabel('Iteration')
        ax3.set_ylabel('Velocity (mm/s)')
        ax3.set_title('Control Velocity (speedL)', fontsize=12, fontweight='bold')
        ax3.grid(True, alpha=0.3)
        
        # XYZ position over time
        ax4 = fig.add_subplot(2, 2, 4)
        ax4.plot(iterations, poses[:-1, 0], 'r-', label='X', linewidth=2)
        ax4.plot(iterations, poses[:-1, 1], 'g-', label='Y', linewidth=2)
        ax4.plot(iterations, poses[:-1, 2], 'b-', label='Z', linewidth=2)
        ax4.axhline(y=self.target_pose[0], color='r', linestyle='--', alpha=0.5)
        ax4.axhline(y=self.target_pose[1], color='g', linestyle='--', alpha=0.5)
        ax4.axhline(y=self.target_pose[2], color='b', linestyle='--', alpha=0.5)
        ax4.set_xlabel('Iteration')
        ax4.set_ylabel('Position (m)')
        ax4.set_title('Position Components', fontsize=12, fontweight='bold')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(filename, dpi=150, bbox_inches='tight')
        print(f"📊 Saved visualization to {filename}")
        plt.close()
    
    def visualize_snapshots(self, filename='visual_servo_snapshots.png'):
        """Create snapshots at different stages"""
        fig, axes = plt.subplots(1, 4, figsize=(20, 5), subplot_kw={'projection': '3d'})
        
        poses = np.array(self.pose_history)
        n_poses = len(poses)
        snapshot_indices = [0, n_poses//3, 2*n_poses//3, n_poses-1]
        titles = ['Start', 'Early', 'Mid', 'Final']
        
        for ax, idx, title in zip(axes, snapshot_indices, titles):
            # Current pose
            current = poses[idx]
            
            # Plot target
            ax.scatter(self.target_pose[0], self.target_pose[1], self.target_pose[2],
                      c='red', s=300, marker='*', label='Target', alpha=0.8)
            
            # Plot current
            ax.scatter(current[0], current[1], current[2],
                      c='blue', s=200, marker='o', label='Robot', alpha=0.8)
            
            # Draw line to target
            ax.plot([current[0], self.target_pose[0]],
                   [current[1], self.target_pose[1]],
                   [current[2], self.target_pose[2]],
                   'k--', alpha=0.5, linewidth=2)
            
            # Draw coordinate frame
            scale = 0.08
            colors = ['red', 'green', 'blue']
            for i, color in enumerate(colors):
                vec = np.zeros(3)
                vec[i] = scale
                arrow = Arrow3D([current[0], current[0]+vec[0]],
                              [current[1], current[1]+vec[1]],
                              [current[2], current[2]+vec[2]],
                              mutation_scale=15, lw=2, arrowstyle='->', color=color)
                ax.add_artist(arrow)
            
            error = np.linalg.norm(current[:3] - self.target_pose[:3])
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_zlabel('Z (m)')
            ax.set_title(f'{title}\nError: {error*1000:.1f}mm', 
                        fontsize=12, fontweight='bold')
            ax.legend()
            
            # Set consistent limits
            ax.set_xlim([0.3, 0.6])
            ax.set_ylim([0.2, 0.4])
            ax.set_zlim([0.3, 0.5])
        
        plt.tight_layout()
        plt.savefig(filename, dpi=150, bbox_inches='tight')
        print(f"📸 Saved snapshots to {filename}")
        plt.close()
    
    def create_animation_frames(self, output_dir='animation_frames'):
        """Create individual frames for animation"""
        import os
        os.makedirs(output_dir, exist_ok=True)
        
        poses = np.array(self.pose_history)
        n_poses = len(poses)
        
        # Create frames at regular intervals
        frame_interval = max(1, n_poses // 30)  # Target ~30 frames
        
        for frame_idx, pose_idx in enumerate(range(0, n_poses, frame_interval)):
            fig = plt.figure(figsize=(12, 9))
            ax = fig.add_subplot(111, projection='3d')
            
            current = poses[pose_idx]
            
            # Plot trajectory up to current point
            ax.plot(poses[:pose_idx+1, 0], poses[:pose_idx+1, 1], poses[:pose_idx+1, 2],
                   'b-', linewidth=2, alpha=0.6, label='Trajectory')
            
            # Plot target with coordinate frame
            ax.scatter(self.target_pose[0], self.target_pose[1], self.target_pose[2],
                      c='red', s=400, marker='*', label='Target', alpha=0.8)
            
            # Draw TARGET coordinate frame (larger and more visible)
            scale = 0.12
            colors = ['red', 'green', 'blue']
            labels = ['X', 'Y', 'Z']
            
            # Target frame - use target orientation (0,0,0 in this case)
            from scipy.spatial.transform import Rotation as R
            target_rot = R.from_rotvec(self.target_pose[3:])
            target_rot_matrix = target_rot.as_matrix()
            
            for i, (color, label) in enumerate(zip(colors, labels)):
                vec = np.zeros(3)
                vec[i] = scale
                # Apply rotation
                vec_rotated = target_rot_matrix @ vec
                arrow = Arrow3D([self.target_pose[0], self.target_pose[0]+vec_rotated[0]],
                              [self.target_pose[1], self.target_pose[1]+vec_rotated[1]],
                              [self.target_pose[2], self.target_pose[2]+vec_rotated[2]],
                              mutation_scale=20, lw=3, arrowstyle='->', color=color, alpha=0.7)
                ax.add_artist(arrow)
            
            # Plot current position
            ax.scatter(current[0], current[1], current[2],
                      c='blue', s=300, marker='o', label='Robot', alpha=0.9)
            
            # Draw CURRENT coordinate frame (larger and rotated)
            current_rot = R.from_rotvec(current[3:])
            current_rot_matrix = current_rot.as_matrix()
            
            for i, (color, label) in enumerate(zip(colors, labels)):
                vec = np.zeros(3)
                vec[i] = scale
                # Apply rotation
                vec_rotated = current_rot_matrix @ vec
                arrow = Arrow3D([current[0], current[0]+vec_rotated[0]],
                              [current[1], current[1]+vec_rotated[1]],
                              [current[2], current[2]+vec_rotated[2]],
                              mutation_scale=20, lw=3, arrowstyle='->', color=color, alpha=0.9)
                ax.add_artist(arrow)
            
            # Draw line to target
            ax.plot([current[0], self.target_pose[0]],
                   [current[1], self.target_pose[1]],
                   [current[2], self.target_pose[2]],
                   'k--', alpha=0.5, linewidth=2)
            
            position_error = np.linalg.norm(current[:3] - self.target_pose[:3])
            rotation_error = np.linalg.norm(current[3:] - self.target_pose[3:])
            
            ax.set_xlabel('X (m)', fontsize=11)
            ax.set_ylabel('Y (m)', fontsize=11)
            ax.set_zlabel('Z (m)', fontsize=11)
            ax.set_title(f'Visual Servoing: Iteration {pose_idx}\n'
                        f'Position Error: {position_error*1000:.1f}mm | '
                        f'Rotation Error: {rotation_error:.3f}rad', 
                        fontsize=14, fontweight='bold')
            ax.legend(loc='upper right', fontsize=10)
            
            # Set consistent limits
            ax.set_xlim([0.3, 0.6])
            ax.set_ylim([0.2, 0.4])
            ax.set_zlim([0.3, 0.5])
            
            # Adjust viewing angle for better visibility
            ax.view_init(elev=20, azim=45)
            
            # Save frame
            frame_path = os.path.join(output_dir, f'frame_{frame_idx:03d}.png')
            plt.savefig(frame_path, dpi=100, bbox_inches='tight')
            plt.close()
        
        print(f"🎬 Created {frame_idx+1} animation frames in {output_dir}/")
        return frame_idx + 1

def main():
    """Run visual servoing simulation"""
    print("=" * 60)
    print("Visual Servoing Simulation with speedL Control")
    print("Demonstrates PBVS with Cartesian velocity commands")
    print("=" * 60)
    print()
    
    # Create and run simulation
    sim = VisualServoSimulation()
    converged = sim.run_simulation(max_iters=50, tolerance=0.002)
    
    print()
    print("=" * 60)
    print("Generating visualizations...")
    print("=" * 60)
    
    # Generate visualizations
    sim.visualize_trajectory('visual_servo_trajectory.png')
    sim.visualize_snapshots('visual_servo_snapshots.png')
    
    # Create animation frames
    print()
    print("=" * 60)
    print("Creating animation...")
    print("=" * 60)
    n_frames = sim.create_animation_frames('animation_frames')
    
    # Create GIF from frames
    try:
        import imageio
        import glob
        
        frames = []
        frame_files = sorted(glob.glob('animation_frames/frame_*.png'))
        
        print(f"📹 Compiling {len(frame_files)} frames into GIF...")
        for frame_file in frame_files:
            frames.append(imageio.imread(frame_file))
        
        # Save as GIF with good quality
        imageio.mimsave('visual_servo_animation.gif', frames, 
                       duration=0.1, loop=0)
        print("✅ Saved animation to visual_servo_animation.gif")
        
        # Clean up frame files
        import shutil
        shutil.rmtree('animation_frames')
        print("🧹 Cleaned up temporary frames")
        
    except ImportError:
        print("⚠️  imageio not available - install with: pip install imageio")
        print("   Animation frames saved in animation_frames/ directory")
    except Exception as e:
        print(f"⚠️  Could not create GIF: {e}")
        print("   Animation frames saved in animation_frames/ directory")
    
    print()
    print("✅ Simulation complete!")
    print()
    print("Key Findings:")
    print(f"  • Converged: {converged}")
    print(f"  • Iterations: {len(sim.error_history)}")
    print(f"  • Final error: {sim.error_history[-1]*1000:.2f}mm")
    print(f"  • Initial error: {sim.error_history[0]*1000:.2f}mm")
    print(f"  • Control gain: {sim.control_gain}")
    print(f"  • Control rate: {1/sim.dt:.0f} Hz")
    print()
    print("This demonstrates how speedL() enables smooth")
    print("convergence through Cartesian velocity control.")

if __name__ == "__main__":
    main()
