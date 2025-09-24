# AprilTag-Based Robot Vision Workflow

Complete guide for setting up AprilTag detection and pick-and-place workflows with UR robot and Pi camera.

## Table of Contents
1. [Hardware Setup](#hardware-setup)
2. [Camera Server Setup](#camera-server-setup)
3. [Camera Calibration](#camera-calibration)
4. [Hand-Eye Calibration](#hand-eye-calibration)
5. [AprilTag Detection Setup](#apriltag-detection-setup)
6. [Grasp Teaching](#grasp-teaching)
7. [Pick and Place Deployment](#pick-and-place-deployment)
8. [Troubleshooting](#troubleshooting)

---

## Hardware Setup

### Required Hardware
- **UR Robot** (tested with UR5/UR10)
- **Raspberry Pi Camera** (Pi Camera v2 or compatible)
- **Wrist-mounted camera bracket** (positioned near tool flange)
- **AprilTags** (printed on quality paper, 23mm recommended)
- **Gripper** with known TCP offset (166mm in our setup)

### Camera Mounting
1. **Mount camera near robot wrist/tool flange**
   - Position for clear workspace view
   - Minimize occlusion during grasping
   - Secure cable routing to prevent snagging

2. **Camera Orientation**
   - Camera should look toward workspace
   - Z-axis pointing "forward" from camera
   - Maintain consistent mounting for calibration

---

## Camera Server Setup

### Pi Camera Server Installation
```bash
# On Raspberry Pi - ensure internet connection and install packages
pip install picamera2 flask pyyaml

# Start camera server (one-line command)
python -m pi_cam_server
```

### Client Configuration
Edit network settings in YAML files:

**`client_config.yaml`:**
```yaml
camera:
  server_ip: "192.168.1.10"  # Pi IP address
  server_port: 8000
  timeout: 5.0
  
capture:
  resolution: [1920, 1080]
  format: "jpeg"
  quality: 95
```

### Network Setup
**Important Network Requirements:**
- **Robot + Laptop**: Must be on same subnet (e.g., 192.168.0.x)
  - Robot: `192.168.0.10`
  - Laptop: `192.168.0.100`
- **Pi Camera + Laptop**: Must be on same subnet (e.g., 192.168.1.x)  
  - Pi Camera: `192.168.1.10`
  - Laptop: `192.168.1.100` (secondary interface)
- **Robot + Pi Camera**: Do NOT need to be on same subnet

**Network Configuration:**
- Configure IP addresses in respective YAML files
- Laptop may need multiple network interfaces
- Test connectivity with ping before proceeding

### Test Camera Connection
```bash
cd tests
python test_camera_capture.py
```

---

## Camera Calibration

### Capture Calibration Images
```bash
cd camera_calibration
python capture_calibration_photos.py --camera-config ../client_config.yaml
```

**Calibration Tips:**
- Use provided chessboard pattern (US Letter size)
- Capture 15-20 images with varied orientations
- Fill entire camera view with chessboard
- Avoid motion blur and ensure good lighting

### Calculate Camera Intrinsics
```bash
python calculate_camera_intrinsics.py
```

**Output:** `camera_calibration.yaml` containing:
- Camera matrix (focal length, principal point)
- Distortion coefficients
- Calibration quality metrics

---

## Hand-Eye Calibration

### Prerequisites
1. **Robot coordinate system set to "Base"** (on teach pendant)
2. **AprilTag mounted on wall or table** (wall mounting works fine)
3. **Camera server running** and accessible

### Critical Setup Notes
- **Coordinate Frame**: Ensure robot teach pendant shows "Base" coordinates
- **RTDE Interface**: Script automatically handles coordinate frame differences
- **Sign Corrections**: Built-in Ry/Rz sign corrections for UR robots

### Data Collection
```bash
cd handeye_calibration
python collect_handeye_data.py --robot-ip 192.168.0.10
```

**Collection Process:**
1. Put robot in **FREEDRIVE mode** for safety
2. Manually move robot to diverse poses (15+ recommended)
3. Ensure AprilTag visible in each pose
4. **Vary orientations significantly** between poses
5. Press Enter to capture each pose
6. Press 'q' when finished

**Pose Diversity Requirements:**
- Large rotational changes (30+ degrees between poses)
- Different orientations around X, Y, Z axes
- Keep AprilTag clearly visible and well-lit
- Avoid duplicate poses (major cause of calibration failure)

### Calculate Hand-Eye Calibration
```bash
python calculate_handeye_calibration.py --input handeye_data_YYYYMMDD_HHMMSS.json
```

**Expected Results:**
- **Translation**: Camera position relative to TCP
  - X, Y: Small offsets (0-50mm typical)
  - Z: Distance from camera to TCP (varies by mounting)
- **Rotation**: Small orientation differences (typically <30°)

**Quality Indicators:**
- Non-identity transformation matrix
- Physically reasonable camera-to-TCP distances
- Consistent results across multiple calibrations

---

## AprilTag Detection Setup

### AprilTag Configuration
```python
apriltag_config = {
    'tag_family': 'tag36h11',       # Standard family
    'tag_size_mm': 23.0,            # Physical tag size
    'calibration_file': 'camera_calibration/camera_calibration.yaml'
}
```

### Tag Printing Guidelines
- **Print at actual size** (23mm for our config)
- **Use quality paper** (matte preferred)
- **Avoid reflections** and ensure flat mounting
- **Multiple tags** can be used for different objects

### Testing Detection
```bash
cd tests
python test_apriltag_detection.py --camera-config ../client_config.yaml
```

---

## Grasp Teaching

### Overview
Teach robot grasp poses relative to AprilTag coordinates, enabling automatic adaptation when objects move.

### Teaching Process

#### 1. Create Grasp Teaching Script
```python
# grasp_teaching.py
from robots.ur.ur_controller import URController
from camera.picam.picam import PiCam
from tests.test_apriltag_detection import AprilTagDetector

class GraspTeacher:
    def __init__(self, robot_ip, camera_config):
        self.robot = URController(robot_ip, read_only=True)
        self.camera = PiCam(camera_config)
        self.detector = AprilTagDetector()
        self.taught_grasps = []
    
    def teach_grasp_interactive(self, tag_id):
        print(f"Teaching grasps for AprilTag {tag_id}")
        print("Instructions:")
        print("1. Put robot in FREEDRIVE mode")
        print("2. Position robot at desired grasp pose")
        print("3. Press Enter to record grasp")
        print("4. Type 'done' when finished")
        
        while True:
            user_input = input("Position robot and press Enter (or 'done'): ")
            if user_input.lower() == 'done':
                break
                
            # Get current robot pose
            robot_pose = self.robot.get_tcp_pose()
            
            # Detect AprilTag
            image_path = self.camera.capture_photo()
            image = cv2.imread(image_path)
            detections = self.detector.detect_tags(image, estimate_pose=True)
            
            # Find target tag
            tag_detection = None
            for det in detections:
                if det['tag_id'] == tag_id:
                    tag_detection = det
                    break
            
            if tag_detection is None:
                print(f"AprilTag {tag_id} not detected, skipping...")
                continue
            
            # Calculate relative grasp pose
            tag_pose = self.extract_tag_pose(tag_detection)
            relative_grasp = self.calculate_relative_pose(robot_pose, tag_pose)
            
            grasp_data = {
                'position': relative_grasp[:3].tolist(),
                'orientation': relative_grasp[3:].tolist(),
                'approach_direction': [0, 0, -1],  # Approach from above
                'quality': 1.0,
                'name': f'grasp_{len(self.taught_grasps) + 1}'
            }
            
            self.taught_grasps.append(grasp_data)
            print(f"Recorded grasp {len(self.taught_grasps)}: {grasp_data['name']}")
    
    def save_grasps(self, filename):
        grasp_config = {
            'tag_id': tag_id,
            'grasps': self.taught_grasps,
            'coordinate_frame': 'tag_relative',
            'teaching_date': datetime.now().isoformat()
        }
        
        with open(filename, 'w') as f:
            json.dump(grasp_config, f, indent=2)
        
        print(f"Saved {len(self.taught_grasps)} grasps to {filename}")
```

#### 2. Home Position Definition
Define observation pose where camera has clear workspace view:

```python
# Define in robot base coordinates
HOME_OBSERVATION_POSE = [-0.2, -0.4, 0.4, 0.0, 1.57, 0.0]  # Example pose

def move_to_observation_pose(robot):
    """Move robot to position for optimal object detection"""
    robot.move_to_pose(HOME_OBSERVATION_POSE)
    time.sleep(1.0)  # Allow settling
```

#### 3. Teaching Workflow
```bash
# Attach AprilTag to object
# Run teaching script
python grasp_teaching.py --robot-ip 192.168.0.10 --tag-id 1

# Follow prompts to teach multiple grasp approaches
# Save taught grasps to file
```

---

## Pick and Place Deployment

### Core Implementation

#### 1. Pose Transformation Utilities
```python
import numpy as np
from scipy.spatial.transform import Rotation as R

def load_hand_eye_calibration(calib_file):
    """Load hand-eye calibration from YAML file"""
    with open(calib_file, 'r') as f:
        calib = yaml.safe_load(f)
    return np.array(calib['camera_to_robot_matrix'])

def transform_camera_to_robot(camera_pose, hand_eye_transform):
    """Transform pose from camera coordinates to robot coordinates"""
    # Convert pose to transformation matrix
    camera_transform = pose_to_transform_matrix(camera_pose)
    
    # Apply hand-eye calibration
    robot_transform = hand_eye_transform @ camera_transform
    
    # Convert back to pose
    return transform_matrix_to_pose(robot_transform)

def transform_relative_to_world(relative_grasp, tag_pose_world):
    """Transform relative grasp pose to world coordinates"""
    tag_transform = pose_to_transform_matrix(tag_pose_world)
    relative_transform = pose_to_transform_matrix(relative_grasp)
    
    world_transform = tag_transform @ relative_transform
    return transform_matrix_to_pose(world_transform)
```

#### 2. Pick and Place Executor
```python
class PickAndPlaceExecutor:
    def __init__(self, robot_ip, camera_config, hand_eye_calib):
        self.robot = URController(robot_ip)
        self.camera = PiCam(camera_config)
        self.detector = AprilTagDetector()
        self.hand_eye_transform = load_hand_eye_calibration(hand_eye_calib)
    
    def execute_pick_and_place(self, tag_id, grasps_file, place_location):
        """Complete pick and place workflow"""
        
        # 1. Move to observation position
        self.robot.move_to_pose(HOME_OBSERVATION_POSE)
        
        # 2. Detect object location
        success, object_pose_robot = self.detect_object_pose(tag_id)
        if not success:
            print("Object detection failed")
            return False
        
        # 3. Load and transform grasps
        with open(grasps_file, 'r') as f:
            grasp_config = json.load(f)
        
        # 4. Attempt pick with each grasp
        for grasp in grasp_config['grasps']:
            grasp_pose_robot = transform_relative_to_world(grasp, object_pose_robot)
            
            if self.execute_pick(grasp_pose_robot):
                # 5. Execute place
                return self.execute_place(place_location)
        
        print("All grasps failed")
        return False
    
    def detect_object_pose(self, tag_id):
        """Detect object pose via AprilTag"""
        image_path = self.camera.capture_photo()
        image = cv2.imread(image_path)
        detections = self.detector.detect_tags(image, estimate_pose=True)
        
        for det in detections:
            if det['tag_id'] == tag_id:
                # Transform from camera to robot coordinates
                camera_pose = self.extract_tag_pose(det)
                robot_pose = transform_camera_to_robot(camera_pose, self.hand_eye_transform)
                return True, robot_pose
        
        return False, None
    
    def execute_pick(self, grasp_pose):
        """Execute picking motion"""
        try:
            # Pre-grasp pose (approach from above)
            pre_grasp = grasp_pose.copy()
            pre_grasp[2] += 0.1  # 10cm above
            
            # Motion sequence
            self.robot.move_to_pose(pre_grasp)
            self.robot.move_to_pose(grasp_pose)
            self.robot.close_gripper()
            self.robot.move_to_pose(pre_grasp)  # Lift
            
            return True
        except Exception as e:
            print(f"Pick failed: {e}")
            return False
    
    def execute_place(self, place_location):
        """Execute placing motion"""
        try:
            # Pre-place pose
            pre_place = place_location.copy()
            pre_place[2] += 0.1
            
            # Motion sequence
            self.robot.move_to_pose(pre_place)
            self.robot.move_to_pose(place_location)
            self.robot.open_gripper()
            self.robot.move_to_pose(pre_place)
            
            return True
        except Exception as e:
            print(f"Place failed: {e}")
            return False
```

### Usage Example
```python
# Initialize system
executor = PickAndPlaceExecutor(
    robot_ip='192.168.0.10',
    camera_config='../client_config.yaml',
    hand_eye_calib='handeye_calibration/handeye_calibration_latest.yaml'
)

# Define place location
PLACE_LOCATION = [-0.1, -0.3, 0.2, 0.0, 1.57, 0.0]

# Execute pick and place
success = executor.execute_pick_and_place(
    tag_id=1,
    grasps_file='grasps_tag_1.json',
    place_location=PLACE_LOCATION
)

print(f"Pick and place {'succeeded' if success else 'failed'}")
```

---

## Troubleshooting

### Common Issues and Solutions

#### Camera Calibration Problems
**Symptoms:** Poor detection accuracy, distorted poses
**Solutions:**
- Recapture calibration images with better lighting
- Ensure chessboard fills camera view
- Check for motion blur
- Verify chessboard pattern is flat

#### Hand-Eye Calibration Issues
**Symptoms:** Identity matrix results, unrealistic offsets
**Solutions:**
- Ensure Base coordinate frame on robot
- Increase pose diversity (more varied orientations)
- Check for duplicate poses in data
- Verify AprilTag size matches configuration

#### AprilTag Detection Problems
**Symptoms:** No detections, poor pose estimation
**Solutions:**
- Improve lighting conditions
- Check tag print quality and flatness
- Verify tag size in configuration
- Ensure camera is in focus

#### Robot Motion Issues
**Symptoms:** Protective stops, unreachable poses
**Solutions:**
- Use slower motion speeds
- Add intermediate waypoints
- Check joint limits and singularities
- Verify workspace boundaries

#### Coordinate Frame Mismatches
**Symptoms:** Objects picked at wrong locations
**Solutions:**
- Verify hand-eye calibration quality
- Check coordinate transformations
- Ensure consistent tag orientation
- Validate pose transformation math

### Debug Tools

#### Real-time Pose Monitoring
```bash
cd robots/ur
python test_robot_pose.py --robot-ip 192.168.0.10 --continuous
```

#### AprilTag Detection Testing
```bash
cd tests
python test_apriltag_detection.py --camera-config ../client_config.yaml --continuous
```

#### Calibration Validation
```bash
cd handeye_calibration
python test_integration.py  # Test complete pipeline
```

---

## System Architecture

### File Structure
```
robot_system_tools/
├── camera/
│   ├── picam/                    # Pi camera client
│   └── __init__.py
├── camera_calibration/
│   ├── capture_calibration_photos.py
│   ├── calculate_camera_intrinsics.py
│   └── camera_calibration.yaml   # Output
├── handeye_calibration/
│   ├── collect_handeye_data.py
│   ├── calculate_handeye_calibration.py
│   └── handeye_calibration_*.yaml # Output
├── robots/ur/
│   ├── ur_robot_interface.py     # Main robot control
│   └── test_robot_pose.py        # Debug utility
├── tests/
│   ├── test_apriltag_detection.py
│   └── test_integration.py
├── client_config.yaml            # Camera client config
└── documentation/
    └── APRILTAG_WORKFLOW.md      # This file
```

### Key Components
- **URController**: Robot control with coordinate frame corrections
- **PiCam**: Camera client for remote Pi camera server
- **AprilTagDetector**: Tag detection and pose estimation
- **Hand-eye calibration**: Camera-to-robot coordinate transformation
- **Grasp teaching**: Relative pose recording and playback

---

## Performance Optimization

### Detection Speed
- Use lower camera resolution for faster processing
- Optimize AprilTag detector parameters
- Consider GPU acceleration for larger deployments

### Motion Planning
- Pre-compute safe trajectories
- Use joint-space planning for faster motion
- Implement collision checking for safety

### Reliability Improvements
- Multiple detection attempts before failure
- Grasp quality scoring and selection
- Error recovery and retry logic
- Sensor fusion with force feedback

---

## Next Steps and Extensions

### Immediate Improvements
1. **Multi-object handling** - detect and grasp multiple tagged objects
2. **Place pose teaching** - teach relative place locations
3. **Grasp quality assessment** - score and rank grasps automatically
4. **Error handling** - robust failure recovery

### Advanced Features
1. **Dynamic obstacle avoidance** - real-time path replanning
2. **Learning from demonstration** - improve grasps through experience
3. **Multi-camera fusion** - combine multiple viewpoints
4. **Force-guided assembly** - precision insertion tasks

### Integration Opportunities
1. **ROS integration** - connect to larger robotic ecosystems
2. **Cloud logging** - remote monitoring and analytics
3. **Digital twin** - simulation and virtual training
4. **Production scaling** - multi-robot coordination

---

## Changelog

- **2025-09-05**: Initial comprehensive workflow documentation
- **2025-09-04**: Hand-eye calibration coordinate frame fixes
- **2025-09-03**: Camera calibration and AprilTag detection setup
- **2025-09-02**: Initial camera server and robot interface development

---

## Support and Maintenance

### Regular Maintenance
- **Recalibrate cameras** quarterly or after mounting changes
- **Verify hand-eye calibration** monthly or after system modifications
- **Update taught grasps** when objects or workflows change
- **Monitor detection quality** and retrain as needed

### System Validation
- Run integration tests before production use
- Verify coordinate transformations with known test objects
- Check motion safety and emergency stops
- Validate workspace boundaries and joint limits

---

*This workflow represents a complete AprilTag-based vision system for robot pick-and-place applications. The modular design allows for easy extension and customization for specific applications.*
