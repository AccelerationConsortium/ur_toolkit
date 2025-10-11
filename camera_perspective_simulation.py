#!/usr/bin/env python3
"""
Camera Perspective Visual Servoing Simulation
Shows how AprilTag appears in camera view during convergence
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrowPatch
from matplotlib import patches
import cv2

class CameraPerspectiveSimulation:
    """Simulates camera view of AprilTag during visual servoing"""
    
    def __init__(self):
        # Camera intrinsics (simulated)
        self.focal_length = 500  # pixels
        self.image_width = 640
        self.image_height = 480
        self.cx = self.image_width / 2
        self.cy = self.image_height / 2
        
        # AprilTag physical size (23mm)
        self.tag_size = 0.023  # meters
        
        # Target pose (where we want the tag to be in camera frame)
        # Tag centered and at ~0.3m distance
        self.target_camera_to_tag = np.array([0.0, 0.0, 0.3, 0.0, 0.0, 0.0])
        
        # Initial pose (offset and rotated)
        self.current_camera_to_tag = np.array([0.05, -0.04, 0.25, 0.5, -0.3, 0.2])
        
        # Control parameters
        self.control_gain = 0.4
        self.max_iters = 50
        
        # History
        self.pose_history = [self.current_camera_to_tag.copy()]
        
    def project_tag_corners(self, camera_to_tag):
        """Project AprilTag corners to image plane"""
        # Tag corners in tag frame (4 corners of square)
        half_size = self.tag_size / 2
        tag_corners_3d = np.array([
            [-half_size, -half_size, 0],
            [ half_size, -half_size, 0],
            [ half_size,  half_size, 0],
            [-half_size,  half_size, 0]
        ])
        
        # Apply rotation
        from scipy.spatial.transform import Rotation as R
        rotation = R.from_rotvec(camera_to_tag[3:])
        rot_matrix = rotation.as_matrix()
        
        # Transform corners
        corners_camera = []
        for corner in tag_corners_3d:
            # Rotate then translate
            corner_rotated = rot_matrix @ corner
            corner_camera = corner_rotated + camera_to_tag[:3]
            corners_camera.append(corner_camera)
        
        corners_camera = np.array(corners_camera)
        
        # Project to image plane (simple pinhole model)
        corners_image = []
        for corner in corners_camera:
            if corner[2] > 0.01:  # Avoid division by zero
                u = self.focal_length * corner[0] / corner[2] + self.cx
                v = self.focal_length * corner[1] / corner[2] + self.cy
                corners_image.append([u, v])
            else:
                corners_image.append([self.cx, self.cy])  # Default to center
        
        return np.array(corners_image)
    
    def render_camera_view(self, camera_to_tag, iteration):
        """Render what camera sees"""
        fig, ax = plt.subplots(figsize=(8, 6))
        
        # Background
        ax.set_xlim(0, self.image_width)
        ax.set_ylim(self.image_height, 0)  # Image coordinates: origin at top-left
        ax.set_aspect('equal')
        ax.set_facecolor('#f0f0f0')
        
        # Draw crosshair (target center)
        ax.plot([self.cx-20, self.cx+20], [self.cy, self.cy], 'g--', linewidth=2, alpha=0.5)
        ax.plot([self.cx, self.cx], [self.cy-20, self.cy+20], 'g--', linewidth=2, alpha=0.5)
        ax.plot(self.cx, self.cy, 'go', markersize=8, alpha=0.5, label='Target Center')
        
        # Project tag corners
        corners = self.project_tag_corners(camera_to_tag)
        
        # Draw tag
        tag_polygon = patches.Polygon(corners, closed=True, 
                                     edgecolor='black', facecolor='white', 
                                     linewidth=3)
        ax.add_patch(tag_polygon)
        
        # Draw inner black square (AprilTag pattern)
        inner_ratio = 0.8
        center = corners.mean(axis=0)
        inner_corners = center + (corners - center) * inner_ratio
        inner_polygon = patches.Polygon(inner_corners, closed=True,
                                       edgecolor='black', facecolor='black',
                                       linewidth=1)
        ax.add_patch(inner_polygon)
        
        # Draw white blocks (simplified AprilTag pattern)
        for i in range(4):
            block_center = center + (corners[i] - center) * 0.6
            block_size = np.linalg.norm(corners[0] - corners[1]) * 0.15
            block = plt.Circle(block_center, block_size, color='white')
            ax.add_patch(block)
        
        # Calculate errors
        position_error = np.linalg.norm(camera_to_tag[:3] - self.target_camera_to_tag[:3])
        rotation_error = np.linalg.norm(camera_to_tag[3:] - self.target_camera_to_tag[3:])
        
        # Center offset in pixels
        tag_center_img = corners.mean(axis=0)
        pixel_offset = np.linalg.norm(tag_center_img - np.array([self.cx, self.cy]))
        
        # Title with info
        ax.set_title(f'Camera View - Iteration {iteration}\n'
                    f'Position Error: {position_error*1000:.1f}mm | '
                    f'Rotation Error: {rotation_error:.3f}rad | '
                    f'Pixel Offset: {pixel_offset:.1f}px',
                    fontsize=12, fontweight='bold')
        
        ax.set_xlabel('Image Width (pixels)', fontsize=10)
        ax.set_ylabel('Image Height (pixels)', fontsize=10)
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
        
        return fig
    
    def step(self):
        """Execute one visual servoing iteration"""
        # Compute error
        pose_error = self.current_camera_to_tag - self.target_camera_to_tag
        
        # Compute velocity command (simple PBVS)
        velocity = -self.control_gain * pose_error
        
        # Apply velocity limits
        max_vel = 0.05
        vel_magnitude = np.linalg.norm(velocity)
        if vel_magnitude > max_vel:
            velocity = velocity * max_vel / vel_magnitude
        
        # Update pose
        dt = 1.0/30.0
        self.current_camera_to_tag += velocity * dt
        self.pose_history.append(self.current_camera_to_tag.copy())
        
        return np.linalg.norm(pose_error[:3])
    
    def run_simulation(self):
        """Run the simulation and save frames"""
        print("🎥 Camera Perspective Simulation")
        print("=" * 50)
        
        import os
        os.makedirs('camera_frames', exist_ok=True)
        
        # Create frames at intervals
        frame_indices = [0, 10, 20, 30, 40, 49]
        
        for iteration in range(self.max_iters):
            error = self.step()
            
            if iteration in frame_indices:
                fig = self.render_camera_view(self.pose_history[iteration], iteration)
                plt.savefig(f'camera_frames/frame_{iteration:02d}.png', 
                           dpi=100, bbox_inches='tight')
                plt.close()
                print(f"📸 Frame {iteration}: Error = {error*1000:.1f}mm")
        
        # Create final frame
        fig = self.render_camera_view(self.pose_history[-1], len(self.pose_history)-1)
        plt.savefig(f'camera_frames/frame_{len(self.pose_history)-1:02d}.png',
                   dpi=100, bbox_inches='tight')
        plt.close()
        
        print(f"✅ Created {len(frame_indices)+1} camera perspective frames")
        
    def create_gif(self):
        """Compile frames into GIF"""
        try:
            import imageio
            import glob
            
            frames = []
            frame_files = sorted(glob.glob('camera_frames/frame_*.png'))
            
            print(f"📹 Compiling {len(frame_files)} frames into GIF...")
            for frame_file in frame_files:
                img = imageio.imread(frame_file)
                # Duplicate each frame multiple times for slower animation
                for _ in range(5):
                    frames.append(img)
            
            # Save as GIF
            imageio.mimsave('camera_perspective_animation.gif', frames,
                           duration=0.2, loop=0)
            print("✅ Saved animation to camera_perspective_animation.gif")
            
            # Clean up
            import shutil
            shutil.rmtree('camera_frames')
            print("🧹 Cleaned up temporary frames")
            
        except Exception as e:
            print(f"⚠️ Could not create GIF: {e}")

def main():
    """Run camera perspective simulation"""
    print("=" * 60)
    print("Camera Perspective Visual Servoing Simulation")
    print("Shows AprilTag detection during convergence")
    print("=" * 60)
    print()
    
    sim = CameraPerspectiveSimulation()
    sim.run_simulation()
    sim.create_gif()
    
    print()
    print("✅ Camera perspective simulation complete!")
    print()
    print("This shows how the AprilTag appears in the camera")
    print("as the robot converges to the target pose using speedL().")

if __name__ == "__main__":
    main()
