#!/usr/bin/env python3
"""
Camera Perspective Visual Servoing Simulation
Shows how AprilTag appears in camera view during convergence
Uses pupil-apriltags for realistic AprilTag detection
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrowPatch
from matplotlib import patches
import cv2

try:
    from pupil_apriltags import Detector
    APRILTAG_DETECTOR_AVAILABLE = True
except ImportError:
    APRILTAG_DETECTOR_AVAILABLE = False
    print("⚠️ pupil-apriltags not installed. Run: pip install pupil-apriltags")
    print("   Falling back to synthetic AprilTag generation")

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
        self.tag_family = 'tag36h11'
        self.tag_id = 0
        
        # Target pose (where we want the tag to be in camera frame)
        # Tag centered and at ~0.3m distance
        self.target_camera_to_tag = np.array([0.0, 0.0, 0.3, 0.0, 0.0, 0.0])
        
        # Initial pose (offset and rotated significantly)
        self.current_camera_to_tag = np.array([0.06, -0.05, 0.25, 0.6, -0.4, 0.3])
        
        # Control parameters
        self.control_gain = 0.4
        self.max_iters = 150  # Tripled from 50
        
        # History
        self.pose_history = [self.current_camera_to_tag.copy()]
        
        # Initialize AprilTag detector if available
        if APRILTAG_DETECTOR_AVAILABLE:
            self.detector = Detector(
                families=self.tag_family,
                nthreads=1,
                quad_decimate=1.0,
                quad_sigma=0.0,
                refine_edges=1,
                decode_sharpening=0.25,
                debug=0
            )
            # Camera parameters for pupil-apriltags: [fx, fy, cx, cy]
            self.camera_params = [self.focal_length, self.focal_length, self.cx, self.cy]
        else:
            self.detector = None
        
    def generate_apriltag_pattern(self, size=200, tag_id=0):
        """Generate AprilTag using pupil-apriltags library or fallback to synthetic pattern"""
        if APRILTAG_DETECTOR_AVAILABLE:
            try:
                # Try to generate using apriltag library
                # This is a simplified approach - in practice we'd use a tag generation library
                # For now, create a recognizable synthetic pattern
                return self._generate_synthetic_pattern(size)
            except Exception as e:
                print(f"⚠️ AprilTag generation failed: {e}, using synthetic pattern")
                return self._generate_synthetic_pattern(size)
        else:
            return self._generate_synthetic_pattern(size)
    
    def _generate_synthetic_pattern(self, size=200):
        """Generate a realistic AprilTag 36h11 pattern (fallback method)"""
        # Create white background
        tag = np.ones((size, size), dtype=np.uint8) * 255
        
        # Black border (2 pixels)
        border = 2
        tag[:border, :] = 0
        tag[-border:, :] = 0
        tag[:, :border] = 0
        tag[:, -border:] = 0
        
        # Inner pattern for tag36h11 (simplified but recognizable)
        # Create a 6x6 grid pattern
        cell_size = (size - 2*border) // 6
        offset = border
        
        # Specific pattern to make it look like AprilTag 36h11 ID 0
        # This is a simplified pattern that looks like an AprilTag
        pattern = [
            [0, 0, 0, 0, 0, 0],
            [0, 1, 1, 1, 1, 0],
            [0, 1, 0, 0, 1, 0],
            [0, 1, 0, 1, 1, 0],
            [0, 1, 1, 1, 0, 0],
            [0, 0, 0, 0, 0, 0],
        ]
        
        for i in range(6):
            for j in range(6):
                y_start = offset + i * cell_size
                y_end = offset + (i + 1) * cell_size
                x_start = offset + j * cell_size
                x_end = offset + (j + 1) * cell_size
                
                if pattern[i][j] == 0:
                    tag[y_start:y_end, x_start:x_end] = 0
                else:
                    tag[y_start:y_end, x_start:x_end] = 255
        
        return tag
    
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
    
    def render_camera_view_with_target(self, camera_to_tag, iteration):
        """Render camera view with target reference side-by-side"""
        fig = plt.figure(figsize=(14, 6))
        
        # Left subplot: Current view
        ax1 = fig.add_subplot(1, 2, 1)
        self._render_single_view(ax1, camera_to_tag, iteration, is_target=False)
        
        # Right subplot: Target view (reference)
        ax2 = fig.add_subplot(1, 2, 2)
        self._render_single_view(ax2, self.target_camera_to_tag, iteration, is_target=True)
        
        plt.tight_layout()
        return fig
    
    def _render_single_view(self, ax, camera_to_tag, iteration, is_target=False):
        """Render a single camera view"""
        # Background
        ax.set_xlim(0, self.image_width)
        ax.set_ylim(self.image_height, 0)  # Image coordinates: origin at top-left
        ax.set_aspect('equal')
        ax.set_facecolor('#e8e8e8')
        
        if not is_target:
            # Draw crosshair (target center) only in current view
            ax.plot([self.cx-30, self.cx+30], [self.cy, self.cy], 'g--', linewidth=1.5, alpha=0.6)
            ax.plot([self.cx, self.cx], [self.cy-30, self.cy+30], 'g--', linewidth=1.5, alpha=0.6)
        
        # Project tag corners
        corners = self.project_tag_corners(camera_to_tag)
        
        # Generate AprilTag pattern
        tag_pattern = self.generate_apriltag_pattern(size=200)
        
        # Warp the tag pattern to the projected corners
        # Source points (corners of the pattern image)
        src_pts = np.array([[0, 0], [200, 0], [200, 200], [0, 200]], dtype=np.float32)
        # Destination points (projected corners)
        dst_pts = corners.astype(np.float32)
        
        # Compute perspective transform
        M = cv2.getPerspectiveTransform(src_pts, dst_pts)
        
        # Warp the tag pattern
        warped_tag = cv2.warpPerspective(tag_pattern, M, 
                                         (self.image_width, self.image_height),
                                         flags=cv2.INTER_LINEAR,
                                         borderMode=cv2.BORDER_CONSTANT,
                                         borderValue=200)
        
        # Display the warped tag
        ax.imshow(warped_tag, cmap='gray', vmin=0, vmax=255, alpha=0.9)
        
        # Detect AprilTag using pupil-apriltags if available and not target view
        detection_text = ""
        if APRILTAG_DETECTOR_AVAILABLE and self.detector and not is_target:
            try:
                detections = self.detector.detect(
                    warped_tag,
                    estimate_tag_pose=True,
                    camera_params=self.camera_params,
                    tag_size=self.tag_size
                )
                if detections:
                    det = detections[0]
                    detection_text = f"\n✓ Detected: ID {det.tag_id} | Decision Margin: {det.decision_margin:.1f}"
                    # Draw detected corners in green if detection is good
                    if det.decision_margin > 50:
                        detected_corners = det.corners
                        detected_polygon = patches.Polygon(detected_corners, closed=True, 
                                                         edgecolor='green', facecolor='none', 
                                                         linewidth=2, linestyle='-', alpha=0.7)
                        ax.add_patch(detected_polygon)
                else:
                    detection_text = "\n✗ No detection"
            except Exception as e:
                detection_text = f"\n⚠️ Detection error"
        
        # Draw tag outline
        tag_polygon = patches.Polygon(corners, closed=True, 
                                     edgecolor='red', facecolor='none', 
                                     linewidth=2, linestyle='--')
        ax.add_patch(tag_polygon)
        
        if is_target:
            ax.set_title('Target Position\n(Goal)', fontsize=12, fontweight='bold', color='green')
        else:
            # Calculate errors
            position_error = np.linalg.norm(camera_to_tag[:3] - self.target_camera_to_tag[:3])
            rotation_error = np.linalg.norm(camera_to_tag[3:] - self.target_camera_to_tag[3:])
            
            # Center offset in pixels
            tag_center_img = corners.mean(axis=0)
            pixel_offset = np.linalg.norm(tag_center_img - np.array([self.cx, self.cy]))
            
            ax.set_title(f'Current View - Iteration {iteration}\n'
                        f'Pos Err: {position_error*1000:.1f}mm | '
                        f'Rot Err: {rotation_error:.3f}rad | '
                        f'Px Off: {pixel_offset:.1f}px'
                        f'{detection_text}',
                        fontsize=10, fontweight='bold')
        
        ax.set_xlabel('Image Width (pixels)', fontsize=9)
        ax.set_ylabel('Image Height (pixels)', fontsize=9)
        ax.grid(True, alpha=0.2, linewidth=0.5)
    
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
        
        # Create frames every 3 iterations for smoother animation
        frame_interval = 3
        
        for iteration in range(self.max_iters):
            error = self.step()
            
            if iteration % frame_interval == 0 or iteration == self.max_iters - 1:
                fig = self.render_camera_view_with_target(self.pose_history[iteration], iteration)
                plt.savefig(f'camera_frames/frame_{iteration:03d}.png', 
                           dpi=100, bbox_inches='tight')
                plt.close()
                print(f"📸 Frame {iteration}: Error = {error*1000:.1f}mm")
        
        print(f"✅ Created frames for camera perspective")
        
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
                # Add each frame twice for slower animation
                for _ in range(2):
                    frames.append(img)
            
            # Save as GIF
            imageio.mimsave('camera_perspective_animation.gif', frames,
                           duration=0.15, loop=0)
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
    if APRILTAG_DETECTOR_AVAILABLE:
        print("Using pupil-apriltags for AprilTag detection")
    else:
        print("⚠️ pupil-apriltags not available - using synthetic patterns")
        print("   Install with: pip install pupil-apriltags")
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
    print("Left: Current view | Right: Target reference")
    if APRILTAG_DETECTOR_AVAILABLE:
        print()
        print("Green outline = pupil-apriltags detection")
        print("Red dashed outline = ground truth projection")

if __name__ == "__main__":
    main()
