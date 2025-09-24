# Robot System Tools# Robot System Tools# Robot System Tools# Robot System Tools# Robot System Tools



A comprehensive Python toolkit for robot control, computer vision, and workflow automation with Universal Robots.



## Quick StartA comprehensive Python toolkit for robot control, computer vision, and workflow automation with Universal Robots.



### Position Teaching

```bash

python positions/teach_positions.py## Quick StartA comprehensive Python toolkit for robot control, computer vision, and workflow automation with Universal Robots.

```

Interactive tool for teaching and managing robot positions with remote freedrive capability.



### Workflow Execution### Position Teaching

```bash

# Run sample workflow```bash

python workflow/run_workflow.py

python teach_positions.py## Quick StartA comprehensive Python toolkit for robot control, computer vision, and workflow automation with Universal Robots.Complete robot vision system with UR robot control, camera capture, and AprilTag detection.

# Run custom workflow

python workflow/run_workflow.py workflow/examples/sample_workflow.yaml```

```

Interactive tool for teaching and managing robot positions with remote freedrive capability.

## Project Structure



```

├── workflow/                  # Workflow execution system### Workflow Execution### Position Teaching

│   ├── examples/             # Sample workflow YAML files

│   ├── workflow_executor.py  # Core workflow execution engine```bash

│   └── run_workflow.py       # CLI workflow runner

├── positions/                # Robot position files# Run sample workflow```bash

│   ├── taught_positions.yaml # Saved robot positions

│   └── teach_positions.py    # Interactive position teaching toolpython workflow/run_workflow.py

├── setup/                    # Project setup and configuration

│   ├── config.yaml           # System configurationpython teach_positions.py## Quick Start## 🚀 Quick Start

│   ├── config_manager.py     # Configuration management

│   ├── requirements.txt      # Python dependencies# Run custom workflow

│   └── setup_venv.sh         # Virtual environment setup script

├── documentation/            # Detailed documentationpython workflow/run_workflow.py workflow/examples/sample_workflow.yaml```

│   └── CHANGELOG.md          # Project changelog

├── camera/                   # Camera interface modules```

├── camera_calibration/       # Camera calibration tools

├── robots/                   # Robot-specific implementationsInteractive tool for teaching and managing robot positions with remote freedrive capability.

└── tests/                    # Test files

```## Project Structure



## Key Features



- **Interactive Position Teaching** - Remote freedrive with automatic safe offset positioning```

- **YAML Workflow System** - Sequential robot operations with step-by-step execution

- **AprilTag Integration** - Computer vision-based positioning and calibration├── workflow/                  # Workflow execution system### Workflow Execution### Position Teaching### 1. Virtual Environment Setup

- **Camera Calibration** - Tools for camera intrinsic calibration

- **Robot Control** - Universal Robots interface with gripper support│   ├── examples/             # Sample workflow YAML files



## Documentation│   ├── workflow_executor.py  # Core workflow execution engine```bash



See the `documentation/` directory for detailed guides:│   └── run_workflow.py       # CLI workflow runner



- [Workflow System](documentation/WORKFLOW_SYSTEM.md) - Complete workflow usage guide├── positions/                # Robot position files# Run sample workflow```bash

- [Position Teaching](documentation/POSITION_TEACHING.md) - Position teaching workflow

- [AprilTag Workflow](documentation/APRILTAG_WORKFLOW.md) - Computer vision integration│   └── taught_positions.yaml # Saved robot positions

- [Configuration Guide](documentation/CONFIGURATION_GUIDE.md) - System setup and config

- [Changelog](documentation/CHANGELOG.md) - Project change history├── config/                   # Configuration filespython workflow/run_workflow.py



## Requirements│   └── config.yaml           # System configuration



See `setup/requirements.txt` for Python dependencies. Compatible with Universal Robots and Robotiq grippers.├── camera/                   # Camera interface modulespython teach_positions.py**First, set up the complete virtual environment** (required for all components):

├── camera_calibration/       # Camera calibration tools

├── robots/                   # Robot-specific implementations# Run custom workflow

├── documentation/            # Detailed documentation

├── tests/                    # Test filespython workflow/run_workflow.py workflow/examples/sample_workflow.yaml```

└── teach_positions.py        # Interactive position teaching

``````



## Key FeaturesInteractive tool for teaching and managing robot positions with remote freedrive capability.```bash



- **Interactive Position Teaching** - Remote freedrive with automatic safe offset positioning## Project Structure

- **YAML Workflow System** - Sequential robot operations with step-by-step execution

- **AprilTag Integration** - Computer vision-based positioning and calibration# Clone the repo

- **Camera Calibration** - Tools for camera intrinsic calibration

- **Robot Control** - Universal Robots interface with gripper support```



## Documentation├── workflow/                  # Workflow execution system### Workflow Executiongit clone https://github.com/kelvinchow23/robot_system_tools.git



See the `documentation/` directory for detailed guides:│   ├── examples/             # Sample workflow YAML files



- [Workflow System](documentation/WORKFLOW_SYSTEM.md) - Complete workflow usage guide│   ├── workflow_executor.py  # Core workflow execution engine```bashcd robot_system_tools

- [Position Teaching](documentation/POSITION_TEACHING.md) - Position teaching workflow

- [AprilTag Workflow](documentation/APRILTAG_WORKFLOW.md) - Computer vision integration│   └── run_workflow.py       # CLI workflow runner

- [Configuration Guide](documentation/CONFIGURATION_GUIDE.md) - System setup and config

├── camera/                   # Camera interface modules# Run sample workflow

## Requirements

├── camera_calibration/       # Camera calibration tools

See `requirements.txt` for Python dependencies. Compatible with Universal Robots and Robotiq grippers.
├── robots/                   # Robot-specific implementationspython workflow/run_workflow.py# Set up virtual environment with all dependencies

├── documentation/            # Detailed documentation

├── tests/                    # Test files./setup_venv.sh

├── teach_positions.py        # Interactive position teaching

├── config.yaml              # System configuration# Run custom workflow

└── taught_positions.yaml    # Saved robot positions

```python workflow/run_workflow.py workflow/examples/sample_workflow.yaml# Activate the environment



## Key Features```source venv/bin/activate



- **Interactive Position Teaching** - Remote freedrive with automatic safe offset positioning```

- **YAML Workflow System** - Sequential robot operations with step-by-step execution

- **AprilTag Integration** - Computer vision-based positioning and calibration## Project Structure

- **Camera Calibration** - Tools for camera intrinsic calibration

- **Robot Control** - Universal Robots interface with gripper supportThis installs dependencies for:



## Documentation```- UR Robot control (RTDE)



See the `documentation/` directory for detailed guides:├── workflow/                  # Workflow execution system- Camera systems (OpenCV, Pi Camera)  



- [Workflow System](documentation/WORKFLOW_SYSTEM.md) - Complete workflow usage guide│   ├── examples/             # Sample workflow YAML files- AprilTag detection

- [Position Teaching](documentation/POSITION_TEACHING.md) - Position teaching workflow

- [AprilTag Workflow](documentation/APRILTAG_WORKFLOW.md) - Computer vision integration│   ├── workflow_executor.py  # Core workflow execution engine- Development tools

- [Configuration Guide](documentation/CONFIGURATION_GUIDE.md) - System setup and config

│   └── run_workflow.py       # CLI workflow runner

## Requirements

├── camera/                   # Camera interface modules### 2. Configuration

See `requirements.txt` for Python dependencies. Compatible with Universal Robots and Robotiq grippers.
├── camera_calibration/       # Camera calibration tools

├── robots/                   # Robot-specific implementations**Configure the system using the unified configuration file** `config.yaml`:

├── documentation/            # Detailed documentation

├── tests/                    # Test files```yaml

├── teach_positions.py        # Interactive position teaching# Robot Configuration

├── config.yaml              # System configurationrobot:

└── taught_positions.yaml    # Saved robot positions  ip_address: "192.168.0.10"  # Your UR robot IP

```  default_speed: 0.03

  default_acceleration: 0.08

## Key Features

# Camera Configuration  

- **Interactive Position Teaching** - Remote freedrive with automatic safe offset positioningcamera:

- **YAML Workflow System** - Sequential robot operations with step-by-step execution  server:

- **AprilTag Integration** - Computer vision-based positioning and calibration    host: "192.168.1.100"  # Your Pi camera IP

- **Camera Calibration** - Tools for camera intrinsic calibration    port: 2222

- **Robot Control** - Universal Robots interface with gripper support

# AprilTag Configuration

## Documentationapriltag:

  family: "tag36h11"

See the `documentation/` directory for detailed guides:  tag_size: 0.023  # 23mm tags

```

- [Workflow System](documentation/WORKFLOW_SYSTEM.md) - Complete workflow usage guide

- [Position Teaching](documentation/POSITION_TEACHING.md) - Position teaching workflowSee [`documentation/CONFIGURATION_GUIDE.md`](documentation/CONFIGURATION_GUIDE.md) for complete configuration options.

- [AprilTag Workflow](documentation/APRILTAG_WORKFLOW.md) - Computer vision integration

- [Configuration Guide](documentation/CONFIGURATION_GUIDE.md) - System setup and config### 3. Pi Camera Server Setup (One Command)



## RequirementsOn your Raspberry Pi, run:

```bash

See `requirements.txt` for Python dependencies. Compatible with Universal Robots and Robotiq grippers.curl -sSL https://raw.githubusercontent.com/kelvinchow23/robot_system_tools/master/pi_cam_server/install.sh | bash
```

This will:
- Clone this repo
- Install all dependencies using system packages
- Set up camera server as systemd service
- Enable auto-start on boot
- Start the service immediately

### 4. Client Setup

After setting up the virtual environment and configuration:

```bash
# Activate environment (if not already active)
source venv/bin/activate

# Configure your Pi's IP address
# Edit camera_client_config.yaml and set your Pi's IP

# Test the camera connection
python tests/test_camera_capture.py

# Test UR robot connection (update IP in robots/ur/robot_config.yaml)
python tests/test_ur_robot.py --robot-ip 192.168.0.10

# Test AprilTag detection
python tests/test_apriltag_detection.py
```

## 📱 Core Workflows

### 1. Camera Capture

```python
from camera.picam.picam import PiCam, PiCamConfig

# Load config and capture photo
config = PiCamConfig.from_yaml("camera_client_config.yaml")
cam = PiCam(config)
photo_path = cam.capture_photo()

if photo_path:
    print(f"Photo saved: {photo_path}")
```

### 2. AprilTag Detection

```python
from apriltag_detection import AprilTagDetector

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
from robots.ur.ur_robot_interface import URRobotInterface

# Connect to robot
robot = URRobotInterface('192.168.0.10')

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
# 1. Print the calibration chessboard (8x6 external corners, 30mm squares)
# Use: camera_calibration/Calibration chessboard (US Letter).pdf
# Print at 100% scale, mount on rigid surface

# 2. Capture 10+ calibration photos
cd camera_calibration
python capture_calibration_photos.py

# 3. Calculate camera intrinsics
python calculate_camera_intrinsics.py

# 4. Verify calibration quality
# Check camera_calibration.yaml for low reprojection error (<0.5 pixels)
```

## 📁 File Structure

```
robot_system_tools/
├── .github/
│   └── copilot-instructions.md      # Development practices for Copilot
├── documentation/
│   ├── ARCHITECTURE.md              # System architecture
│   └── CHANGELOG.md                 # Project changelog
├── camera/
│   └── picam/                       # Pi camera client
│       ├── picam.py                 # Camera client library
│       └── setup_picam_client.sh    # Client setup script
├── robots/
│   └── ur/                          # Universal Robots interface
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
