# Position Teaching System

The position teaching CLI allows you to teach robot positions using **freedrive mode** that are associated with AprilTag markers placed on physical equipment for grasping/releasing operations.

## Features

- **Freedrive Teaching**: Use manual robot positioning with freedrive mode for intuitive teaching
- **AprilTag Integration**: Automatically associate positions with visible AprilTag markers
- **Move to Positions**: Navigate robot to previously taught positions  
- **Verify Positions**: Check that expected AprilTags are visible
- **Position Management**: List, delete, and organize taught positions
- **Interactive Mode**: Real-time position teaching interface

## Quick Start

### 1. Teach a Position with Freedrive

```bash
# Teach a position using freedrive mode for manual positioning
python teach_positions.py teach "pickup_station" --description "Station where parts are picked up"
```

This will:
- **Enable freedrive mode** - you can manually move the robot arm
- Prompt you to position the robot at the desired location
- **Disable freedrive mode** when positioning is complete
- Record robot pose (position + orientation)
- Capture camera image and detect visible AprilTags
- Save position with AprilTag associations
- Store data in `taught_positions.yaml`

### 2. List Taught Positions

```bash
python teach_positions.py list
```

Shows all taught positions with:
- Position name and description
- Number of associated AprilTags
- Timestamp when taught
- Usage statistics

### 3. Move to Position

```bash
python teach_positions.py move "pickup_station"
```

Moves robot to the taught position with safety confirmation.

### 4. Verify Position

```bash
python teach_positions.py verify "pickup_station"
```

Checks that expected AprilTags are visible from current position.

## Interactive Mode

For continuous position teaching:

```bash
python teach_positions.py interactive
```

Interactive commands:
- `teach <name> [description]` - Teach current position
- `move <name>` - Move to position
- `verify <name>` - Verify position
- `list` - Show all positions
- `delete <name>` - Remove position
- `quit` - Exit

## Position Data Structure

Each taught position includes:

```yaml
position_name:
  name: "pickup_station"
  description: "Station where parts are picked up"
  robot_pose: [x, y, z, rx, ry, rz]  # TCP pose in meters/radians
  joint_positions: [j1, j2, j3, j4, j5, j6]  # Joint angles in radians
  apriltag_detections:
    - tag_id: 5
      distance_mm: 450.2
      pose: {...}  # 3D pose of detected tag
  image_path: "photos/capture_20250916_123456.jpg"
  taught_timestamp: "2025-09-16T12:34:56"
  usage_count: 3
  last_used: "2025-09-16T14:22:10"
```

## Configuration

Configure position teaching in `config.yaml`:

```yaml
position_teaching:
  auto_save: true  # Automatically save after teaching
  require_apriltag: false  # Require AprilTag for teaching
  confirmation_required: true  # Confirm before movements
  backup_positions: true  # Backup positions file

paths:
  taught_positions_file: "taught_positions.yaml"
```

## Workflow Example

### Setting up Pickup/Drop Stations

1. **Place AprilTag markers** on physical equipment (workstations, fixtures, etc.)

2. **Teach pickup position**:
   ```bash
   python teach_positions.py teach "pickup_station_A" --description "Pick parts from conveyor"
   ```
   
   **Freedrive Teaching Process**:
   - Robot enters freedrive mode automatically
   - **Manually move the robot arm** to the desired pickup position
   - Press **ENTER** when positioned correctly
   - Robot captures scene and detects AprilTags
   - Position is saved with AprilTag associations

3. **Teach approach position** (safe position before pickup):
   ```bash
   python teach_positions.py teach "approach_pickup_A" --description "Safe approach to pickup station"
   ```

4. **Teach drop position**:
   ```bash
   python teach_positions.py teach "drop_station_B" --description "Drop parts at assembly station"
   ```

5. **Verify positions work**:
   ```bash
   python teach_positions.py verify "pickup_station_A"
   python teach_positions.py move "approach_pickup_A"
   python teach_positions.py move "pickup_station_A"
   ```

### Production Use

In your automation scripts:

```python
from teach_positions import PositionTeacher

teacher = PositionTeacher()
teacher.connect()

# Execute pick-and-place sequence
teacher.move_to_position("approach_pickup_A")
teacher.move_to_position("pickup_station_A")
# ... grasp operation ...
teacher.move_to_position("drop_station_B")
# ... release operation ...
```

## Safety Features

- **Confirmation prompts** before robot movements
- **Position verification** using AprilTag visibility
- **Movement validation** with pose tolerance checking
- **Usage tracking** for position reliability monitoring

## Safety Guidelines for Freedrive Mode

**⚠️ IMPORTANT SAFETY CONSIDERATIONS**

When using freedrive mode for position teaching:

1. **Clear Work Area**: Ensure the robot's workspace is clear of people and obstacles
2. **Manual Control**: You have direct physical control of the robot arm - move slowly and deliberately  
3. **Emergency Stop**: Keep the robot's emergency stop button within easy reach
4. **Joint Limits**: Be aware of joint limits and avoid forcing the robot beyond its range
5. **Smooth Movements**: Move the robot smoothly to avoid triggering protective stops
6. **Stable Positioning**: Ensure the robot is in a stable position before confirming
7. **Tool Awareness**: Be mindful of any tools or end-effectors attached to the robot

**✅ Best Practices**:
- Start with small, controlled movements
- Support the robot arm weight when moving vertically
- Test position stability before confirming
- Use two hands for better control of heavier robot arms

## Command Reference

```bash
# Teach position using freedrive mode
python teach_positions.py teach <name> [--description "text"]

# List all taught positions
python teach_positions.py list

# Move robot to taught position
python teach_positions.py move <name>

# Verify position by checking AprilTag visibility
python teach_positions.py verify <name>

# Delete taught position
python teach_positions.py delete <name>

# Interactive mode for multiple operations
python teach_positions.py interactive

# Override connection settings
python teach_positions.py --robot-ip 192.168.1.10 --camera-host 192.168.1.20 <command>
```

The position teaching system provides a robust foundation for creating repeatable robot operations with visual verification through AprilTag markers.