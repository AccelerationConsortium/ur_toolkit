# Configuration Management Guide

## Overview

The Robot System Tools now uses a unified configuration system with a single `config.yaml` file in the project root. This eliminates the need for multiple config files and provides a centralized way to manage all system settings.

## Configuration File Structure

The main configuration file `config.yaml` contains all settings organized into logical sections:

- **`system`**: General system information and environment settings
- **`robot`**: Robot connection, movement, and safety parameters
- **`camera`**: Camera server settings and local client configuration
- **`apriltag`**: AprilTag detection and pose estimation settings
- **`coordinates`**: Coordinate system conventions and frame definitions
- **`paths`**: File and directory paths (relative to project root)
- **`logging`**: Logging configuration and output settings
- **`debug`**: Development and debugging options
- **`safety`**: Safety limits and workspace boundaries

## Using the Configuration System

### Import the Configuration Manager

```python
from config_manager import config

# Or import convenience functions
from config_manager import (
    get_robot_ip, 
    get_camera_host, 
    get_apriltag_family
)
```

### Access Configuration Values

#### Using Dot Notation
```python
# Get specific values using dot notation
robot_ip = config.get('robot.ip_address')
camera_host = config.get('camera.server.host')
tag_size = config.get('apriltag.tag_size')

# With default values
speed = config.get('robot.default_speed', 0.05)
```

#### Using Convenience Functions
```python
# Pre-defined convenience functions
robot_ip = get_robot_ip()
camera_host = get_camera_host()
tag_family = get_apriltag_family()
```

#### Getting Full Sections
```python
# Get entire configuration sections
robot_config = config.get_section('robot')
camera_config = config.get_section('camera')
apriltag_config = config.get_section('apriltag')
```

### Path Resolution

The configuration manager automatically resolves relative paths:

```python
# Automatically resolves relative to project root
calibration_file = config.resolve_path('camera_calibration/camera_calibration.yaml')
photos_dir = config.resolve_path('photos')

# Or use convenience functions
calibration_file = get_camera_calibration_file()
photos_dir = get_photos_directory()
```

## Module Integration Examples

### Robot Interface

```python
from config_manager import config

class URController:
    def __init__(self, robot_ip=None, speed=None):
        # Use provided values or fall back to config
        self.robot_ip = robot_ip or config.get('robot.ip_address')
        self.speed = speed or config.get('robot.default_speed')
```

### AprilTag Detection

```python
from config_manager import get_apriltag_family, get_apriltag_size

class AprilTagDetector:
    def __init__(self, tag_family=None, tag_size=None):
        # Use provided values or fall back to config
        self.tag_family = tag_family or get_apriltag_family()
        self.tag_size = tag_size or get_apriltag_size()
```

### Camera Client

```python
from config_manager import get_camera_host, get_camera_port

# Create camera config from unified config
host = get_camera_host()
port = get_camera_port()
camera_config = PiCamConfig(hostname=host, port=port)
```

## Configuration Overrides

### Command Line Arguments
You can still override config values with command-line arguments:

```python
# Example: Robot interface with IP override
robot = URController(robot_ip="192.168.1.50")

# Example: AprilTag with family override  
detector = AprilTagDetector(tag_family="tag25h9")
```

### Environment Variables
Set the config file location via environment variable:

```bash
export ROBOT_TOOLS_CONFIG=/path/to/custom/config.yaml
```

### Multiple Environments
Create different config files for different environments:

```yaml
# config.yaml (development)
system:
  environment: "development"
robot:
  ip_address: "192.168.0.10"

# config_production.yaml
system:
  environment: "production" 
robot:
  ip_address: "10.0.1.100"
```

Load specific config:
```python
config.reload_config("config_production.yaml")
```

## Configuration Validation

The system provides defaults and graceful fallbacks:

```python
# If config.yaml is missing, uses built-in defaults
robot_ip = config.get('robot.ip_address', '192.168.0.10')

# Check if configuration loaded successfully
if config._config is None:
    print("Warning: Using default configuration")
```

## Best Practices

### 1. Use Centralized Config for New Features
Always add new configuration options to the main `config.yaml`:

```yaml
# Add new feature settings
my_new_feature:
  enabled: true
  timeout: 30
  max_retries: 3
```

### 2. Provide Sensible Defaults
Always provide defaults in your code:

```python
timeout = config.get('my_feature.timeout', 30)  # 30 second default
```

### 3. Make Config Optional
Allow overrides while using config as fallback:

```python
def __init__(self, setting=None):
    self.setting = setting or config.get('feature.setting')
```

### 4. Use Path Resolution
For file paths, use the path resolution feature:

```python
# Automatically resolves relative to project root
data_file = config.resolve_path(config.get('paths.data_file'))
```

### 5. Document Your Config Options
Add comments to the `config.yaml` file:

```yaml
# My Feature Configuration
my_feature:
  enabled: true        # Enable/disable the feature
  timeout: 30          # Timeout in seconds
  max_retries: 3       # Maximum retry attempts
```

## Migration from Old System

The new system is backward compatible but you should update your modules:

### Before (Old System)
```python
# Loading separate config files
with open('robot_config.yaml') as f:
    robot_config = yaml.safe_load(f)

with open('camera_config.yaml') as f:
    camera_config = yaml.safe_load(f)

robot_ip = robot_config['robot']['ip_address']
```

### After (New System)
```python
# Using unified config
from config_manager import config

robot_ip = config.get('robot.ip_address')
```

## Debugging Configuration

### Print Current Configuration
```python
# Print all configuration
config.print_config()

# Print specific section
config.print_config('robot')
```

### Check Configuration Source
```python
# Check what config file was loaded
print(f"Config loaded from: {config.get_config_path()}")

# Check if using defaults
if config._config.get('_using_defaults'):
    print("Warning: Using default configuration")
```

### Test Configuration Loading
```python
# Test the configuration system
python config_manager.py
```

This will print all configuration values and verify the system is working correctly.