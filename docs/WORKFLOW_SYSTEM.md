# Robot Workflow System

A comprehensive system for executing sequential robot operations using YAML workflow definitions.

## Quick Start

### 1. Run the Sample Workflow
```bash
# Put your robot in REMOTE CONTROL mode first!
python workflow/run_workflow.py
```

### 2. Run a Custom Workflow
```bash
python workflow/run_workflow.py workflow/examples/simple_test.yaml
```

### 3. Run Using the Workflow Executor Directly
```bash
python workflow/workflow_executor.py workflow/examples/sample_workflow.yaml
```

## Workflow File Format

Workflows are defined in YAML format with the following structure:

```yaml
name: "Workflow Name"
description: "What this workflow does"
author: "Creator"
created: "2025-09-17"

settings:
  default_speed: 0.1          # Default movement speed (10%)
  safety_height: 0.02         # Safety offset in meters
  gripper_timeout: 5.0        # Gripper operation timeout
  position_tolerance: 0.01    # Position verification tolerance

steps:
  - name: "Step Description"
    action: "action_type"
    # action-specific parameters
```

## Available Actions

### Movement Actions

#### `move_to_position`
Move to a taught position.
```yaml
- name: "Move to Grasp Position"
  action: "move_to_position"
  position: "grasp-A"          # Must exist in taught_positions.yaml
  speed: 0.1                   # Optional: movement speed (0.01-1.0)
```

#### `offset_move`
Move by a relative offset from current position.
```yaml
- name: "Move Up 2cm"
  action: "offset_move"
  offset: [0, 0, 0.02, 0, 0, 0]  # [x, y, z, rx, ry, rz] in meters/radians
  speed: 0.05                     # Optional: movement speed
  coordinate_system: "base"       # "base" or "tcp"
```

#### `home`
Move to a home position.
```yaml
- name: "Go Home"
  action: "home"
  position: "safe-home"        # Optional: specify home position name
  speed: 0.1                   # Optional: movement speed
```

### Gripper Actions

#### `gripper_activate`
Activate the gripper (required before first use).
```yaml
- name: "Activate Gripper"
  action: "gripper_activate"
```

#### `gripper_open`
Open the gripper.
```yaml
- name: "Open Gripper"
  action: "gripper_open"
```

#### `gripper_close`
Close the gripper.
```yaml
- name: "Close Gripper"
  action: "gripper_close"
  wait_time: 2.0               # Optional: wait time after movement
```

#### `gripper_position`
Set gripper to specific position (0=open, 255=closed).
```yaml
- name: "Half Close Gripper"
  action: "gripper_position"
  position: 128                # Position (0-255)
  wait_time: 2.0               # Optional: wait time after movement
```

#### `gripper_params`
Set gripper force and/or speed parameters.
```yaml
- name: "Configure Gripper"
  action: "gripper_params"
  force: 100                   # Optional: force (0-255)
  speed: 150                   # Optional: speed (0-255)
```

### Utility Actions

#### `delay`
Wait for a specified time.
```yaml
- name: "Wait for Stabilization"
  action: "delay"
  duration: 2.0                # Time in seconds
```

#### `verify_position`
Verify robot is at expected position.
```yaml
- name: "Verify Position"
  action: "verify_position"
  position: "grasp-A"          # Position to verify against
  tolerance: 0.01              # Optional: tolerance in meters
```

## Example Workflows

### Simple Pick and Place
```yaml
name: "Pick and Place"
description: "Basic pick and place operation"

steps:
  - name: "Activate Gripper"
    action: "gripper_activate"

  - name: "Go Home"
    action: "home"
    position: "safe-home"

  - name: "Open Gripper"
    action: "gripper_open"

  - name: "Move to Pick Position"
    action: "move_to_position"
    position: "grasp-A"
    speed: 0.05

  - name: "Close Gripper"
    action: "gripper_close"

  - name: "Lift Object"
    action: "offset_move"
    offset: [0, 0, 0.02, 0, 0, 0]

  - name: "Move to Place Position"
    action: "move_to_position"
    position: "grasp-B"

  - name: "Open Gripper"
    action: "gripper_open"

  - name: "Return Home"
    action: "home"
```

## Safety Guidelines

1. **Always put robot in REMOTE CONTROL mode** before running workflows
2. **Test workflows with low speeds** first (speed: 0.05-0.1)
3. **Use safety offsets** when moving between positions
4. **Include home positions** at start and end of workflows
5. **Verify critical positions** using `verify_position` action

## Workflow Files Included

- `workflow/examples/sample_workflow.yaml` - Complete pick and place demo
- `workflows/simple_test.yaml` - Basic connectivity and movement test
- `workflows/inspection_routine.yaml` - Position verification routine

## Troubleshooting

### Common Issues

1. **"Position not found"**: Ensure position exists in `taught_positions.yaml`
2. **"Robot not connected"**: Check robot IP and network connection
3. **"Movement failed"**: Check robot is in remote mode and not in protective stop

### Running in Simulation Mode
For testing without robot hardware, you can modify the workflow executor to use simulation mode.

## Advanced Usage

### Custom Workflow Creation
1. Copy `workflow/examples/sample_workflow.yaml` as a template
2. Modify steps for your specific application
3. Test with slow speeds first
4. Gradually increase speeds as needed

### Integration with Position Teaching
Workflows automatically use positions from your `taught_positions.yaml` file. Teach new positions using:
```bash
python teach_positions.py teach new-position
```

### Workflow Validation
The system validates:
- Required positions exist
- Action parameters are valid
- Movement commands are safe

### Execution History
Access workflow execution history:
```python
executor = WorkflowExecutor()
history = executor.get_workflow_history()
```