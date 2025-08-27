# Hexapod Movement Programming Guide

## Table of Contents
- [Quick Start](#quick-start)
- [Movement Definition System](#movement-definition-system)
  - [Basic Movement Commands](#basic-movement-commands)
  - [Gait Patterns](#gait-patterns)
- [Programming Interface](#programming-interface)
- [Startup Pre-Movement System](#startup-pre-movement-system)
- [Example Configurations](#example-configurations)
- [Integration Guidelines](#integration-guidelines)
- [Technical Specifications](#technical-specifications)
- [Troubleshooting](#troubleshooting)

## Quick Start

### Basic Movement Example
```json
{
  "startup_sequence": [
    {
      "command": "stand",
      "duration": 2000
    },
    {
      "command": "walk_forward",
      "steps": 5,
      "speed": 100
    }
  ],
  "movement_library": {
    "stand": {
      "type": "posture",
      "height": 120,
      "duration": 1000
    },
    "walk_forward": {
      "type": "gait",
      "pattern": "tripod",
      "step_length": 50,
      "step_height": 30
    }
  }
}
```

## Movement Definition System

### Basic Movement Commands

#### Forward/Backward Locomotion
```json
{
  "command": "move",
  "direction": "forward",
  "distance": 100,  // mm
  "speed": 80       // 0-255
}
```

#### Rotation
```json
{
  "command": "rotate",
  "direction": "left",
  "angle": 90,      // degrees
  "speed": 60
}
```

### Gait Patterns

#### Tripod Gait (Default)
```python
{
  "gait": "tripod",
  "cycle_time": 1000,  // ms
  "duty_factor": 0.5,  // 0-1
  "swing_height": 30   // mm
}
```

## Programming Interface

### JSON Configuration Format
```json
{
  "startup_sequence": [
    {
      "command": "calibrate",
      "timeout": 5000
    }
  ],
  "movement_library": {
    "crouch": {
      "type": "posture",
      "height": 80,
      "duration": 1000
    }
  },
  "default_parameters": {
    "speed": 100,
    "acceleration": 50
  }
}
```

## Startup Pre-Movement System

### Initialization Sequence
1. Power-on self-test
2. Servo calibration
3. Center all servos
4. Raise to default height
5. Check battery level
6. Verify sensor readings

## Example Configurations

### Dance Routine
```json
{
  "sequence": [
    {"command": "move_sideways", "direction": "left", "steps": 3},
    {"command": "rotate", "angle": 180},
    {"command": "wave_leg", "leg": "front_right", "cycles": 3}
  ]
}
```

## Integration Guidelines

### Loading Movement Files
```python
import json

def load_movement_file(filename):
    with open(filename, 'r') as f:
        return json.load(f)
```

## Technical Specifications

### Coordinate System
```
      Front
        ^
        |
L1 L2 L3   R1 R2 R3
 \  |  /     \ | /
  \ | /       \|/
   ---         ---
   |||         |||
   ---         ---
  / | \       / | \
 /  |  \     /  |  \
L4 L5 L6   R4 R5 R6
        |
        v
       Back
```

## Troubleshooting

### Common Issues
1. **Jittery Movement**
   - Check power supply
   - Verify servo calibration
   - Reduce movement speed

2. **Leg Collisions**
   - Increase step height
   - Adjust gait timing
   - Check leg range of motion

3. **Battery Issues**
   - Monitor voltage levels
   - Check power connections
   - Reduce movement speed/load

4. **Servo Overheating**
   - Check for mechanical binding
   - Reduce load on servos
   - Allow cool-down period between movements

5. **Communication Errors**
   - Verify wiring connections
   - Check baud rate settings
   - Ensure sufficient power supply

### Error Codes
- `E001`: Servo communication timeout
- `E002`: Battery voltage low
- `E003`: Servo overload
- `E004`: Invalid movement command
- `E005`: Gait pattern error

### Maintenance Schedule
- **Daily**: Check battery levels
- **Weekly**: Inspect servo gears
- **Monthly**: Recalibrate servos
- **As Needed**: Clean and lubricate joints
