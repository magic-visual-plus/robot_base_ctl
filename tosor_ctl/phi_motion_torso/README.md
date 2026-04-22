# Phi Motion Torso

ROS2 package for controlling the torso/waist lift mechanism of the Phi robot.

## Overview

This package provides a ROS2 node that controls a single-axis lift column using CANopen protocol and DS402 state machine. It is designed as a standard actuator node within the Phi robot's motion control system.

## Features

- **Action-based control**: Point-to-point height control via ROS2 action
- **Status publishing**: Continuous status updates at configurable rate
- **Parameter management**: Read and write motion parameters (velocity, acceleration, deceleration)
- **Safety**: Height range validation (230-700mm), single active goal enforcement
- **Modular design**: Separated controller and ROS2 node layers

## Interfaces

### Namespace
All interfaces are under `/phi/motion/torso`

### Action
- `/phi/motion/torso/move_to_height` (phi_motion_torso/action/MoveToHeight)
  - Goal: target height in mm
  - Feedback: current height, position error, in-window status
  - Result: success status, final height and position

### Topic
- `/phi/motion/torso/status` (phi_motion_torso/msg/TorsoStatus)
  - Published at 10 Hz (configurable)
  - Contains: current/target height, position, goal status, PDO counter

### Services
- `/phi/motion/torso/initialize` (phi_motion_torso/srv/Initialize)
  - Initialize CANopen connection and configure drive

- `/phi/motion/torso/get_motion_params` (phi_motion_torso/srv/GetMotionParams)
  - Read current velocity, acceleration, deceleration

- `/phi/motion/torso/set_motion_params` (phi_motion_torso/srv/SetMotionParams)
  - Set motion parameters (0 = keep current value)

## Quick Start

### Build
```bash
cd ~/ros2_ws
colcon build --packages-select phi_motion_torso
source install/setup.bash
```

### Run
```bash
# Start node
ros2 run phi_motion_torso torso_node

# Initialize hardware (in another terminal)
ros2 service call /phi/motion/torso/initialize phi_motion_torso/srv/Initialize

# Move to height
ros2 action send_goal /phi/motion/torso/move_to_height phi_motion_torso/action/MoveToHeight "{target_height_mm: 500.0}" --feedback

# Monitor status
ros2 topic echo /phi/motion/torso/status
```

## Configuration

ROS2 parameters (set via command line or launch file):

- `eds_file`: Path to EDS file (default: `/home/nvidia/robot_base_ctl/motor/moons/CANOPEN-EDS-MBDV-Servo-SingleAxis-V1.1.1.eds`)
- `can_channel`: CAN interface name (default: `can0`)
- `can_bitrate`: CAN bitrate (default: `100000`)
- `node_id`: CANopen node ID (default: `4`)
- `status_publish_rate`: Status publishing frequency in Hz (default: `10.0`)

Example with custom parameters:
```bash
ros2 run phi_motion_torso torso_node --ros-args \
  -p can_channel:=can1 \
  -p node_id:=5 \
  -p status_publish_rate:=20.0
```

## Height Constraints

- **Minimum height**: 230 mm
- **Maximum height**: 700 mm
- **Zero reference**: encoder=0 corresponds to 480 mm
- **Resolution**: 1,000,000 pulses = 34.25 mm

## Safety Features

1. Height range validation (230-700mm)
2. Single active goal enforcement
3. Initialization required before operation
4. Graceful shutdown with proper CANopen cleanup
5. Comprehensive error handling and logging

## Architecture

```
torso_node.py (ROS2 Node)
    ├── Action Server (move_to_height)
    ├── Status Publisher (status)
    ├── Service Servers (initialize, get/set params)
    └── controller.py (CANopen/DS402 Controller)
            ├── CANopen Network Management
            ├── PDO Configuration
            ├── DS402 State Machine
            └── Motion Control
```

## Dependencies

- ROS2 (Humble or later)
- Python 3.8+
- python-canopen
- python-can

## License

Apache-2.0

## Maintainer

Phi Robot Team
