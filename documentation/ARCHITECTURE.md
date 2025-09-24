# Robot Vision System Architecture

## 🎯 Current Status: Simple & Clean

The robot vision system has been simplified and consolidated per user requirements:
- **No duplicates** - All Pi camera server code lives only in `pi_cam_server/`
- **Simple camera class** - `picam.py` provides all camera functions for other scripts
- **Self-contained** - Each script has a single, focused purpose

## 📁 Directory Structure

```
robot_system_tools/
├── 📸 Pi Camera Server (All server code here)
│   ├── pi_cam_server/
│   │   ├── camera_server.py      # Main Pi camera server
│   │   ├── camera-server.service # Systemd service file
│   │   ├── requirements.txt      # Server dependencies
│   │   ├── setup.sh             # Pi deployment script
│   │   └── README.md            # Server documentation
│   
├── 🤖 Client Scripts (Simple & focused)
│   ├── picam.py                 # Simple camera class ⭐ NEW!
│   ├── camera_client.py         # Basic camera client (uses picam.py)
│   ├── test_robot_vision.py     # Complete workflow test (uses picam.py)
│   ├── detect_apriltags.py      # AprilTag detection only
│   ├── calibrate_camera.py      # Camera calibration only
│   └── camera_config.yaml       # Configuration file
│   
├── 📋 Project Files
│   ├── requirements.txt         # Client dependencies
│   ├── deploy_to_pi.sh         # Deploy script
│   └── README.md               # Main documentation
│   
└── 📦 Archive (Legacy code)
    └── archive/                 # Old experiments and reference code
```

## 🔧 Key Components

### 1. PiCam Class (`picam.py`) ⭐ NEW!
**Simple camera class with all necessary functions:**
- `connect()` - Establish connection to Pi camera server
- `capture_photo()` - Take photo and save locally
- `test_connection()` - Check if server is accessible
- `PiCamConfig` - YAML configuration loader

**Usage Example:**
```python
from picam import PiCam, PiCamConfig

# Load config and connect
config = PiCamConfig.from_yaml("camera_config.yaml")
camera = PiCam(config)

# Test connection
if camera.test_connection():
    # Capture photo
    photo_path = camera.capture_photo()
    print(f"Photo saved: {photo_path}")
```

### 2. Pi Camera Server (`pi_cam_server/`)
**Self-contained server code:**
- Runs on Raspberry Pi Zero 2W
- Socket-based photo capture service
- Systemd service integration
- All server code lives here (no duplicates elsewhere)

### 3. Client Scripts
**Simple, focused scripts:**
- `camera_client.py` - Uses PiCam class for basic photo capture
- `test_robot_vision.py` - Uses PiCam class for complete workflow testing
- `detect_apriltags.py` - Focused on AprilTag detection only
- `calibrate_camera.py` - Focused on camera calibration only

## 🚀 Workflow

1. **Setup Pi Camera Server:**
   ```bash
   # Deploy to Pi
   ./deploy_to_pi.sh pi@your_pi_ip
   
   # Or manually
   scp -r pi_cam_server/ pi@your_pi_ip:/home/pi/robot_camera/
   ssh pi@your_pi_ip 'cd /home/pi/robot_camera && sudo ./setup.sh'
   ```

2. **Use Camera from Client:**
   ```python
   # Simple usage
   from picam import PiCam
   camera = PiCam()  # Uses default config
   photo = camera.capture_photo()
   
   # Or with custom config
   config = PiCamConfig(hostname="your-pi-ip", port=2222)
   camera = PiCam(config)
   photo = camera.capture_photo()
   ```

3. **Complete Robot Vision Workflow:**
   ```bash
   # Test complete workflow
   python test_robot_vision.py your_pi_ip
   
   # Detect AprilTags in latest photo
   python detect_apriltags.py --latest --show
   ```

## ✅ Consolidation Complete

**What was removed:**
- `src/camera/` - Complex modular library (overkill for our needs)
- Duplicate server code - All moved to `pi_cam_server/`
- Complex abstractions - Replaced with simple PiCam class

**What was simplified:**
- Camera operations now use simple PiCam class
- All scripts are self-contained and focused
- Configuration through simple YAML files
- No more over-engineering

**Result:**
- ✅ No duplicates - All server code in `pi_cam_server/`
- ✅ Simple camera class - `picam.py` for all camera operations
- ✅ Self-contained scripts - Each has single purpose
- ✅ Easy to maintain - Clear separation of concerns
- ✅ Easy to extend - Add new scripts using PiCam class

The system is now exactly as requested: simple, focused, and consolidated with no duplicate server code.
