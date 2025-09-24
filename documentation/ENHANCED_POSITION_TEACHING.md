# Enhanced Position Teaching Workflow

## Overview

The robot position teaching system has been enhanced to provide a streamlined and user-friendly workflow for teaching positions with comprehensive metadata collection and automatic observation pose management.

## Key Features

### 1. Rich Metadata Collection
When teaching any position, the system interactively collects:
- **Description**: Detailed description of the position
- **Equipment Name**: Name of the equipment or workpiece
- **Position Type**: Type of position (grasp, place, approach, etc.)
- **Priority**: Priority level (low, normal, high, critical)
- **Safety Notes**: Any safety considerations or warnings

### 2. Automatic Observation Pose Workflow
When teaching a position without AprilTag visibility:
- System automatically prompts to teach an observation pose
- Guides user through the observation pose teaching process
- Automatically links the position to the new observation pose
- Ensures positions have proper AprilTag correction capabilities

### 3. Smart Position Linking
- Positions can be linked to existing observation poses
- Automatic linking when teaching new observation poses
- Manual linking via the `link` command

## Workflow Examples

### Teaching a Grasp Position (Enhanced Workflow)

```bash
# Teach a new position
python teach_positions.py teach grasp-bolt-A

# System prompts for metadata:
# Position description: Grasp bolt A on conveyor
# Equipment name: conveyor-station-1
# Position type (default: grasp): grasp
# Priority (low/normal/high/critical, default: normal): high
# Safety notes (optional): Ensure conveyor is stopped

# If no AprilTags detected, system prompts:
# Would you like to teach an observation pose for AprilTag correction? (Y/N)

# If Y selected, guides through observation pose teaching:
# Observation pose name (default: grasp-bolt-A-obs): 
# Equipment name (required): conveyor-station-1
# Observation pose description (optional): View AprilTags on conveyor
```

### Complete Position Data Structure

The enhanced system stores comprehensive metadata:

```yaml
positions:
  grasp-bolt-A:
    coordinates: [0.5, 0.2, 0.3, 0.0, 3.14, 0.0]
    joints: [45.0, -90.0, 90.0, -90.0, -90.0, 0.0]
    description: "Grasp bolt A on conveyor"
    equipment: "conveyor-station-1"
    position_type: "grasp"
    priority: "high"
    safety_notes: "Ensure conveyor is stopped"
    tag_reference: null
    has_apriltag_view: false
    camera_to_tag: null
    observation_pose: "grasp-bolt-A-obs"
    pose_type: "position"
    timestamp: "2025-01-15T10:30:00"
    
  grasp-bolt-A-obs:
    coordinates: [0.4, 0.1, 0.5, 0.0, 3.14, 0.0]
    joints: [30.0, -80.0, 80.0, -90.0, -90.0, 0.0]
    description: "View AprilTags on conveyor"
    tag_reference: "tag_2"
    has_apriltag_view: true
    camera_to_tag: {...}
    observation_pose: null
    equipment_name: "conveyor-station-1"
    pose_type: "observation"
    timestamp: "2025-01-15T10:32:00"
```

## Interactive Metadata Collection

The system provides guided prompts for all metadata fields:

1. **Position Description**: Free-text description of what the position does
2. **Equipment Name**: Name/ID of equipment or workpiece (helps with organization)
3. **Position Type**: 
   - `grasp` - Picking up objects
   - `place` - Placing objects down
   - `approach` - Approach positions before grasp/place
   - `inspect` - Inspection or measurement positions
   - `custom` - User-defined type
4. **Priority**: 
   - `low` - Non-critical positions
   - `normal` - Standard operations (default)
   - `high` - Important positions
   - `critical` - Safety-critical or essential positions
5. **Safety Notes**: Optional field for safety warnings or special considerations

## Benefits

1. **Complete Documentation**: Every position includes rich metadata for maintenance and operation
2. **Safety Integration**: Safety notes are captured and stored with positions
3. **Equipment Tracking**: Clear association between positions and equipment/workpieces
4. **Priority Management**: Priority levels help with task scheduling and execution
5. **Streamlined Workflow**: Automatic prompting reduces manual steps and errors
6. **AprilTag Integration**: Seamless integration of observation poses for AprilTag correction

## Usage Tips

- Always provide meaningful descriptions and equipment names
- Use priority levels to indicate critical vs. routine positions
- Include safety notes for any positions with special considerations
- Let the system guide you through observation pose teaching for best results
- Review the position list regularly to ensure metadata is current and accurate
- Use consistent equipment naming conventions for better organization

## Benefits

1. **Streamlined workflow** - No need to separately teach observation poses
2. **Intelligent prompting** - System guides you through the process
3. **Automatic linking** - Positions are automatically associated
4. **Equipment tracking** - Clear linkage between poses and equipment
5. **Flexible options** - Choose existing observation poses or create new ones

## File Structure

After using this workflow, your `taught_positions.yaml` will contain:

```yaml
positions:
  grasp-station-A:
    # ... position data ...
    observation_pose: grasp-station-A-obs
    pose_type: position
    
  grasp-station-A-obs:
    # ... observation pose data ...
    equipment_name: "station-A"
    pose_type: observation
    has_apriltag_view: true
    tag_reference: tag_5
```

This creates a complete AprilTag-enabled position system with minimal user effort!