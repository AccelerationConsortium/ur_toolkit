# UR Toolkit

A comprehensive Python toolkit for robot control, computer vision, and workflow automation with Universal Robots.

## 🚀 Quick Start

### 1. Setup
```bash
# Clone the repository
git clone https://github.com/AccelerationConsortium/ur_toolkit.git
cd ur_toolkit

# Set up virtual environment with all dependencies
bash setup/setup_venv.sh
```

### 2. Position Teaching
```bash
python scripts/teach_positions.py
```
Interactive tool for teaching and managing robot positions with remote freedrive capability.

### 3. Workflow Execution
```bash
# Run sample workflow
python scripts/run_workflow.py

# Run custom workflow
python scripts/run_workflow.py examples/workflows/sample_workflow.yaml
```
Complete robot vision system with UR robot control, camera capture, and AprilTag detection.

## 📁 Project Structure

```
ur_toolkit/
├── src/
│   └── ur_toolkit/           # Main Python package
│       ├── camera/           # Camera interface modules
│       ├── robots/           # Robot-specific implementations
│       ├── workflow/         # Workflow execution system
│       ├── visual_servo/     # Visual servoing engine
│       ├── positions/        # Position management
│       ├── camera_calibration/ # Camera calibration tools
│       ├── apriltag_detection.py # AprilTag detection
│       └── config_manager.py # Configuration management
├── scripts/                  # Executable CLI scripts
│   ├── teach_positions.py    # Interactive position teaching
│   └── run_workflow.py       # Workflow runner
├── config/                   # Configuration files
│   └── config.yaml           # System configuration
├── examples/                 # Example workflows
│   └── workflows/            # Sample workflow YAML files
├── docs/                     # Documentation
├── tests/                    # Test files
├── pi_cam_server/           # Camera server (separate deployment)
└── setup/                   # Setup and installation scripts
```

## ✨ Key Features

- **Interactive Position Teaching** - Remote freedrive with automatic safe offset positioning
- **YAML Workflow System** - Sequential robot operations with step-by-step execution
- **AprilTag Integration** - Computer vision-based positioning and calibration
- **Camera Calibration** - Tools for camera intrinsic calibration
- **Robot Control** - Universal Robots interface with gripper support
- **Visual Servoing** - PID-based iterative pose correction

## 📚 Documentation

See the `docs/` directory for detailed guides:

- [Workflow System](docs/WORKFLOW_SYSTEM.md) - Complete workflow usage guide
- [Position Teaching](docs/POSITION_TEACHING.md) - Position teaching workflow
- [AprilTag Workflow](docs/APRILTAG_WORKFLOW.md) - Computer vision integration
- [Configuration Guide](docs/CONFIGURATION_GUIDE.md) - System setup and config
- [Changelog](docs/CHANGELOG.md) - Project change history

## 🔧 Requirements

See `setup/requirements.txt` for Python dependencies. Compatible with Universal Robots and Robotiq grippers.

## 🚀 Usage Examples

### Basic Position Teaching
```bash
# Start interactive position teaching
python scripts/teach_positions.py

# Teach positions for AprilTag ID 5
python scripts/teach_positions.py --tag-id 5
```

### Running Workflows
```bash
# Execute a specific workflow file
python scripts/run_workflow.py examples/workflows/visual_servo_test.yaml

# Run in step-by-step mode
python scripts/run_workflow.py examples/workflows/sample_workflow.yaml --step-mode
```

### Configuration

**Configure the system using the unified configuration file** `config/config.yaml`:

```yaml
# Robot Configuration
robot:
  ip_address: "192.168.0.10"  # Your UR robot IP
  default_speed: 0.03
  default_acceleration: 0.08

# Camera Configuration  
camera:
  server:
    host: "192.168.1.100"  # Your Pi camera IP
    port: 2222

# AprilTag Configuration
apriltag:
  family: "tag36h11"
  tag_size: 0.023  # 23mm tags
```

See [`docs/CONFIGURATION_GUIDE.md`](docs/CONFIGURATION_GUIDE.md) for complete configuration options.

### Pi Camera Server Setup

On your Raspberry Pi, run:

```bash
curl -sSL https://raw.githubusercontent.com/AccelerationConsortium/ur_toolkit/main/pi_cam_server/install.sh | bash
```

This will:
- Clone this repo
- Install all dependencies using system packages
- Set up camera server as systemd service
- Enable auto-start on boot
- Start the service immediately

### Client Setup

After setting up the virtual environment and configuration:

```bash
# Activate environment (if not already active)
source venv/bin/activate

# Test the camera connection
python tests/test_camera_capture.py

# Test UR robot connection
python tests/test_ur_robot.py --robot-ip 192.168.0.10

# Test AprilTag detection
python tests/test_apriltag_detection.py
```

## 📱 Core Workflows

### 1. Camera Capture

```python
from ur_toolkit.camera.picam.picam import PiCam, PiCamConfig

# Load config and capture photo
config = PiCamConfig.from_yaml("config/config.yaml")
cam = PiCam(config)
photo_path = cam.capture_photo()

if photo_path:
    print(f"Photo saved: {photo_path}")
```

### 2. AprilTag Detection

```python
from ur_toolkit.apriltag_detection import AprilTagDetector

# Initialize detector
detector = AprilTagDetector(
    tag_family='tag36h11',
    tag_size=0.023,  # 23mm tags
    camera_calibration_file='camera_calibration/camera_calibration.yaml'
)

# Detect tags in image
import cv2
image = cv2.imread('photo.jpg')
detections = detector.detect_tags(image)

for detection in detections:
    print(f"Tag {detection['tag_id']} at distance {detection['distance']:.3f}m")
    print(f"Position: {detection['pose']['tvec']}")
    print(f"Orientation: {detection['pose']['rvec']}")
```

### 3. Robot Control

```python
from ur_toolkit.robots.ur.ur_controller import URController

# Connect to robot
robot = URController('192.168.0.10')

# Get current pose
current_pose = robot.get_tcp_pose()
print(f"TCP Position: {current_pose[:3]}")

# Move to new position (relative)
new_pose = current_pose.copy()
new_pose[2] += 0.1  # Move up 10cm
robot.move_to_pose(new_pose)
```

### 4. Camera Calibration

For accurate AprilTag pose estimation:

```bash
# 1. Print the calibration chessboard pattern
# 2. Capture calibration photos  
python src/ur_toolkit/camera_calibration/capture_calibration_photos.py

# 3. Calculate camera intrinsics
python src/ur_toolkit/camera_calibration/calculate_camera_intrinsics.py
```

## 🏗️ Development

The project uses a `src/` layout for better packaging and testing:

- **Source code** is in `src/ur_toolkit/`
- **Executable scripts** are in `scripts/`
- **Configuration** is in `config/`
- **Examples** are in `examples/`
- **Tests** import from installed package for accuracy

For development installations:
```bash
pip install -e .
```

## 📝 License

See LICENSE file for details.
│       └── ur_robot_interface.py    # RTDE-based UR interface
├── tests/                           # Test scripts
│   ├── test_camera_capture.py       # Basic camera test
│   ├── test_apriltag_detection.py   # AprilTag detection test
│   ├── test_robot_vision.py         # Complete vision system test
│   └── test_ur_robot.py             # UR robot interface test
├── pi_cam_server/                   # Pi camera server
│   ├── camera_server.py             # Main server application
│   ├── camera_config.yaml           # Server configuration
│   ├── setup.sh                     # Pi setup script
│   ├── install.sh                   # One-line installer
│   └── requirements.txt             # Python dependencies
├── camera_calibration/              # Camera calibration workflow
│   ├── capture_calibration_photos.py  # Capture calibration images
│   ├── calculate_camera_intrinsics.py # Calculate intrinsics
│   ├── camera_calibration.yaml        # Generated camera intrinsics
│   ├── Calibration chessboard (US Letter).pdf  # Chessboard pattern
│   ├── QUALITY_GUIDE.md               # Quality metrics guide
│   └── README.md                      # Calibration documentation
├── camera_client_config.yaml        # Client configuration
└── README.md                        # This file
```

## 🤖 Robot Vision Workflow

### Complete AprilTag Detection and Robot Control

#### 1. Camera Server Setup
On your Raspberry Pi:
```bash
curl -sSL https://raw.githubusercontent.com/kelvinchow23/robot_system_tools/master/pi_cam_server/install.sh | bash
```

#### 2. Test Camera Connection
Edit `camera_client_config.yaml` with your Pi's IP address, then test:
```bash
python tests/test_camera_capture.py
```

#### 3. Camera Calibration
```bash
cd camera_calibration
python capture_calibration_photos.py
python calculate_camera_intrinsics.py
```
This creates `camera_calibration.yaml` with camera intrinsic parameters for accurate pose estimation.

#### 4. AprilTag Detection
```bash
python tests/test_apriltag_detection.py
```
Test AprilTag detection and pose estimation with your calibrated camera.

#### 5. Robot Integration
```python
# Complete robot vision workflow
from robots.ur.ur_robot_interface import URRobotInterface
from camera.picam.picam import PiCam, PiCamConfig
from apriltag_detection import AprilTagDetector

# Initialize systems
robot = URRobotInterface('192.168.0.10')
camera = PiCam(PiCamConfig.from_yaml('camera_client_config.yaml'))
detector = AprilTagDetector(
    tag_family='tag36h11',
    tag_size=0.023,
    camera_calibration_file='camera_calibration/camera_calibration.yaml'
)

# Capture and analyze
photo_path = camera.capture_photo()
image = cv2.imread(photo_path)
detections = detector.detect_tags(image)

# Use detection results for robot control
for detection in detections:
    print(f"AprilTag {detection['tag_id']} detected")
    print(f"Distance: {detection['distance']:.3f}m")
    print(f"Position: {detection['pose']['tvec']}")
    # Implement your robot control logic here
```

## 🔧 Configuration

### Network Setup
- **Robot + Laptop**: Same subnet (e.g., 192.168.0.x)
- **Pi Camera + Laptop**: Same subnet (configure in `camera_client_config.yaml`)

### AprilTag Settings
- Default: tag36h11 family, 23mm size
- Customize in detection code for your specific tags
- Ensure tags are printed at exact scale for accurate pose estimation  
- **Robot + Pi Camera**: Different subnets OK

## 💻 Dependencies

```bash
# Main dependencies (installed by setup_venv.sh)
pip install opencv-python numpy scipy ur-rtde requests pyyaml pupil-apriltags
```

### Development Tools
```bash
# Additional development dependencies
pip install pytest black flake8 mypy
```

## 🛠️ Troubleshooting

### Camera Connection Issues
- Verify Pi IP address in `camera_client_config.yaml`
- Check network connectivity: `ping <pi-ip>`
- Ensure camera server is running on Pi: `systemctl status camera-server`

### AprilTag Detection Issues
- Ensure camera is calibrated (`camera_calibration.yaml` exists)
- Verify tag size matches physical measurement
- Check lighting conditions and tag visibility
- Use `--continuous` mode for real-time debugging

### Robot Connection Issues
- Verify robot IP address
- Check robot is in remote control mode
- Ensure robot safety system is active
- Test with minimal robot movements first

## 📚 Documentation

- [`camera_calibration/README.md`](camera_calibration/README.md) - Camera calibration guide
- [`documentation/ARCHITECTURE.md`](documentation/ARCHITECTURE.md) - System architecture
- [`pi_cam_server/README.md`](pi_cam_server/README.md) - Pi camera server setup

## 🤝 Contributing

See [`.github/copilot-instructions.md`](.github/copilot-instructions.md) for development practices and coding standards.
```yaml
server:
  host: "192.168.1.100"  # Your Pi's IP
  port: 2222
client:
  download_directory: "photos"
  timeout: 10
```

## 🔧 Server Management

On the Pi:

```bash
# Check status
sudo systemctl status camera-server

# View logs
sudo journalctl -u camera-server -f

# Restart service
sudo systemctl restart camera-server

# Stop/start service
sudo systemctl stop camera-server
sudo systemctl start camera-server
```

## 🖥️ Supported Hardware

- **Raspberry Pi Zero 2W** ✅ Tested
- **Raspberry Pi 5** ✅ Tested  
- **Pi Camera v1/v2/v3** ✅ All supported
- **USB Cameras** ✅ Via libcamera

## 🏗️ Architecture

Simple TCP protocol on port 2222:
1. Client connects to Pi server
2. Sends "CAPTURE" command
3. Server captures photo and returns image data
4. Client saves photo locally

The system uses systemd for reliability and auto-start.

## 🔍 Troubleshooting

### Pi Server Issues
```bash
# Check service status
sudo systemctl status camera-server

# View error logs
sudo journalctl -u camera-server -n 50

# Test camera hardware
rpicam-still --timeout 1 -o test.jpg
```

### Client Connection Issues
```bash
# Test connectivity
ping your-pi-ip

# Check port access
telnet your-pi-ip 2222
```

### Common Solutions
- **Camera not found**: Enable camera with `sudo raspi-config`
- **Service won't start**: Check logs and camera hardware
- **Connection refused**: Verify Pi IP in `client_config.yaml`
- **Permission denied**: Ensure setup script ran with proper permissions

## 📄 License

MIT License
