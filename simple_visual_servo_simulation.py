"""
Simplified Visual Servoing Simulation using moveL commands.

This simulation demonstrates a gradient-descent approach to visual servoing
that relies on UR robot's robust inverse kinematics via moveL commands.
No complex Jacobian or velocity control - just iterative position corrections.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation
import urllib.request
import cv2


class SimplifiedVisualServoSimulator:
    """Simulates visual servoing using moveL with gradient-descent approach."""
    
    def __init__(self, control_gain=0.3, max_iterations=100):
        self.control_gain = control_gain
        self.max_iterations = max_iterations
        
        # Initial and target poses (x, y, z, rx, ry, rz) in meters and radians
        self.current_pose = np.array([0.1, -0.05, 0.3, 0.5, -0.3, 0.2])
        self.target_pose = np.array([0.0, 0.0, 0.3, 0.0, 0.0, 0.0])
        
        # Camera parameters
        self.focal_length = 500.0  # pixels
        self.img_width = 640
        self.img_height = 480
        self.tag_size = 0.1  # 10cm AprilTag
        
        # History for visualization
        self.pose_history = [self.current_pose.copy()]
        self.error_history = []
        
        # Download AprilTag image
        self.tag_image = self._load_apriltag_image()
        
    def _load_apriltag_image(self):
        """Load official AprilTag image."""
        url = "https://raw.githubusercontent.com/AprilRobotics/apriltag-imgs/master/tag36h11/tag36_11_00000.png"
        try:
            with urllib.request.urlopen(url) as response:
                img_array = np.asarray(bytearray(response.read()), dtype=np.uint8)
                img = cv2.imdecode(img_array, cv2.IMREAD_GRAYSCALE)
                # Resize to reasonable size
                img = cv2.resize(img, (200, 200), interpolation=cv2.INTER_NEAREST)
                return img
        except Exception as e:
            print(f"Warning: Could not download AprilTag image: {e}")
            # Create synthetic pattern
            img = np.ones((200, 200), dtype=np.uint8) * 255
            img[20:180, 20:180] = 0
            img[60:140, 60:140] = 255
            return img
    
    def compute_pose_error(self):
        """Compute 6-DOF pose error."""
        return self.target_pose - self.current_pose
    
    def moveL(self, target_pose):
        """
        Simulate moveL command - UR robot moves to target pose using inverse kinematics.
        This is the actual robot command we'll use in production.
        """
        # In simulation, we just update the current pose
        # In real robot, this would send RTDE moveL command
        self.current_pose = target_pose.copy()
    
    def visual_servo_step(self):
        """
        One iteration of simplified visual servoing using moveL.
        Instead of velocity commands, we compute a target pose and use moveL.
        """
        # 1. Detect AprilTag and compute pose error
        pose_error = self.compute_pose_error()
        error_magnitude = np.linalg.norm(pose_error[:3])
        self.error_history.append(error_magnitude)
        
        # 2. Compute correction using proportional control
        correction = self.control_gain * pose_error
        
        # 3. Compute new target pose
        new_target_pose = self.current_pose + correction
        
        # 4. Execute moveL to new target pose
        self.moveL(new_target_pose)
        
        # 5. Save history
        self.pose_history.append(self.current_pose.copy())
        
        return error_magnitude
    
    def run_servo_loop(self):
        """Run the visual servoing loop."""
        print("Starting simplified visual servoing simulation...")
        print(f"Initial error: {np.linalg.norm(self.current_pose - self.target_pose):.4f}m")
        
        for i in range(self.max_iterations):
            error = self.visual_servo_step()
            
            if i % 10 == 0:
                print(f"Iteration {i}: Error = {error:.4f}m")
            
            # Check convergence
            if error < 0.005:  # 5mm threshold
                print(f"Converged after {i+1} iterations!")
                break
        
        final_error = np.linalg.norm(self.current_pose - self.target_pose)
        print(f"Final error: {final_error:.4f}m")
        
    def project_tag_to_camera(self, pose):
        """Project AprilTag corners to camera image plane."""
        # Define tag corners in tag frame
        half_size = self.tag_size / 2
        tag_corners_3d = np.array([
            [-half_size, -half_size, 0],
            [half_size, -half_size, 0],
            [half_size, half_size, 0],
            [-half_size, half_size, 0]
        ])
        
        # Transform to camera frame
        position = pose[:3]
        rotation = Rotation.from_rotvec(pose[3:]).as_matrix()
        
        # Transform corners
        corners_camera = []
        for corner in tag_corners_3d:
            corner_world = rotation @ corner + position
            corners_camera.append(corner_world)
        
        corners_camera = np.array(corners_camera)
        
        # Project to image plane
        corners_2d = []
        for corner in corners_camera:
            x, y, z = corner
            if z > 0.01:  # Avoid division by zero
                u = self.focal_length * x / z + self.img_width / 2
                v = self.focal_length * y / z + self.img_height / 2
                corners_2d.append([u, v])
            else:
                corners_2d.append([self.img_width/2, self.img_height/2])
        
        return np.array(corners_2d, dtype=np.float32)
    
    def generate_animation(self, output_file='simple_visual_servo_animation.gif'):
        """Generate animated GIF showing convergence from camera perspective."""
        print(f"\nGenerating animation with {len(self.pose_history)} frames...")
        
        # Create figure with two subplots
        fig, (ax_cam, ax_target) = plt.subplots(1, 2, figsize=(12, 5))
        
        # Sample frames (every 3rd frame for reasonable file size)
        frame_indices = list(range(0, len(self.pose_history), 3))
        if len(self.pose_history) - 1 not in frame_indices:
            frame_indices.append(len(self.pose_history) - 1)
        
        frames = []
        
        for idx in frame_indices:
            pose = self.pose_history[idx]
            error = self.error_history[idx] if idx < len(self.error_history) else 0
            
            # Create current view
            ax_cam.clear()
            ax_cam.set_xlim(0, self.img_width)
            ax_cam.set_ylim(self.img_height, 0)
            ax_cam.set_aspect('equal')
            ax_cam.set_facecolor('white')
            
            # Project tag
            corners = self.project_tag_to_camera(pose)
            
            # Warp AprilTag image
            target_corners = np.array([
                [0, 0],
                [self.tag_image.shape[1], 0],
                [self.tag_image.shape[1], self.tag_image.shape[0]],
                [0, self.tag_image.shape[0]]
            ], dtype=np.float32)
            
            M = cv2.getPerspectiveTransform(target_corners, corners)
            warped = cv2.warpPerspective(self.tag_image, M, (self.img_width, self.img_height))
            
            ax_cam.imshow(warped, cmap='gray', extent=[0, self.img_width, self.img_height, 0])
            
            # Draw corners
            corners_closed = np.vstack([corners, corners[0]])
            ax_cam.plot(corners_closed[:, 0], corners_closed[:, 1], 'g-', linewidth=2, label='Current')
            
            # Add crosshair at center
            cx, cy = self.img_width/2, self.img_height/2
            ax_cam.plot([cx-20, cx+20], [cy, cy], 'g-', linewidth=1)
            ax_cam.plot([cx, cx], [cy-20, cy+20], 'g-', linewidth=1)
            
            ax_cam.set_title(f'Current View - Iter {idx}\nError: {error*1000:.1f}mm')
            ax_cam.axis('off')
            
            # Create target view
            ax_target.clear()
            ax_target.set_xlim(0, self.img_width)
            ax_target.set_ylim(self.img_height, 0)
            ax_target.set_aspect('equal')
            ax_target.set_facecolor('white')
            
            # Project target tag
            target_corners_2d = self.project_tag_to_camera(self.target_pose)
            
            # Warp target AprilTag
            M_target = cv2.getPerspectiveTransform(target_corners, target_corners_2d)
            warped_target = cv2.warpPerspective(self.tag_image, M_target, (self.img_width, self.img_height))
            
            ax_target.imshow(warped_target, cmap='gray', extent=[0, self.img_width, self.img_height, 0])
            
            target_corners_closed = np.vstack([target_corners_2d, target_corners_2d[0]])
            ax_target.plot(target_corners_closed[:, 0], target_corners_closed[:, 1], 'b-', linewidth=2)
            
            ax_target.set_title('Target Reference')
            ax_target.axis('off')
            
            plt.tight_layout()
            
            # Save frame
            fig.canvas.draw()
            image = np.frombuffer(fig.canvas.tostring_rgb(), dtype='uint8')
            image = image.reshape(fig.canvas.get_width_height()[::-1] + (3,))
            frames.append(image)
        
        plt.close(fig)
        
        # Save as GIF
        print(f"Saving animation to {output_file}...")
        import imageio
        imageio.mimsave(output_file, frames, duration=0.2, loop=0)
        print(f"Animation saved! Size: {len(frames)} frames")


def main():
    """Run the simplified visual servoing simulation."""
    # Create simulator with higher gain for faster convergence
    sim = SimplifiedVisualServoSimulator(control_gain=0.5, max_iterations=150)
    
    # Run visual servoing
    sim.run_servo_loop()
    
    # Generate animation
    sim.generate_animation('simple_visual_servo_animation.gif')
    
    print("\nSimulation complete!")
    print("Key insight: This approach uses moveL commands with UR's robust")
    print("inverse kinematics instead of complex velocity control or Jacobians.")
    print("Each iteration computes a target pose correction and uses moveL to get there.")


if __name__ == "__main__":
    main()
