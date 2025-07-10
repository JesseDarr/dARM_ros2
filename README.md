# dARM ROS2 Package

A containerized ROS2 control package for the dARM 6DOF robotic arm, supporting both Gazebo simulation and physical hardware control through ODrive controllers.

## Overview

This repository provides the ROS2 implementation for the dARM robot, featuring:
- Dual-container architecture (simulation on x86, robot on ARM)
- PS5 controller teleop support
- Gazebo simulation with physics
- Hardware interface for ODrive communication (in development)

## Architecture

### Joint Configuration
- **link_1_joint**: Base rotation
- **link_2_joint**: Dual-motor differential drive (2x ODrive nodes)
- **link_3_joint**: Single motor
- **forearm_joint**: Single motor
- **Forearm differential** (bend/twist via 2 motors):
  - `differential_joint`: Bend axis (motors oppose)
  - `gripper_joint`: Twist axis (motors together)
- **Gripper fingers**: Prismatic joints with coupled motion

### Container Strategy
- **Simulation Container**: x86, Ubuntu 24.04, ROS2 Jazzy, Gazebo Sim
- **Robot Container**: arm64, Ubuntu 24.04, ROS2 Jazzy, Bluetooth support

## Current Status

### Implemented ✅
- Complete URDF/Xacro robot description with accurate inertial properties
- Containerized build system with multi-architecture support
- Gazebo simulation environment with physics
- PS5 controller teleop with:
  - Joint position control
  - Predefined poses (home, vert, left, right)
  - Dead-man switch safety
  - Real-time joint state display
- ROS2 control framework with joint trajectory controller
- Launch files for both simulation and hardware
- Transmissions properly configured for differential joints

### In Development 🚧
- ODrive hardware interface plugin (`odrive_ros2_control_plugin/ODriveHardwareInterface`)
- Hardware bring-up procedures

### Planned 📋
- MoveIt2 integration for motion planning
- Computer vision integration

## Prerequisites

### Development Environment
- Docker Engine (Linux) or Docker Desktop (Windows/Mac)
- Git

### For Simulation (Windows WSL2)
- WSL2 with Ubuntu 24.04
- VcXsrv or similar X server

## Usage

### Simulation Mode

```bash
# Using docker-compose
docker-compose -f docker/compose.sim.yml up
```

### Robot Mode

```bash
# Using docker-compose
docker-compose -f docker/compose.robot.yml up
```

## PS5 Controller Mapping

| Control | Function | Notes |
|---------|----------|-------|
| L2 (Hold) | Dead-man switch | Must hold to enable motion |
| Left Stick Y | Joint 0 or 3 or 5 | Depends on bumpers |
| Right Stick X | Joint 1 or 3 or 4 | Depends on bumpers |
| No Bumpers | Control joints 0 & 1 | Base and first link |
| L1 | Control joints 2 & 3 | Upper arm joints |
| R1 | Control joints 4 & 5 | Differential (bend/twist) |
| D-Pad Left/Right | Gripper open/close | Fine control |
| Cross (X) | Home pose | All zeros |
| Triangle | Vertical pose | Arm straight up |
| Square | Left pose | Predefined left position |
| Circle | Right pose | Predefined right position |

## Development Notes

### Container Hardware Passthrough
- **WSL2 GPU**: Uses `/dev/dxg` device mounting
- **USB Controllers**: Requires usbipd-win on Windows host

### Differential Joint Implementation
The forearm differential is implemented using coupled transmissions:
- Both motors contribute to each axis of motion
- Bend motion: motors oppose (different signs in transmission)
- Twist motion: motors work together (same signs)
- Effective gear ratio approaches 13:1 when both motors active

## Troubleshooting

### Controller Not Detected
```bash
# Check device exists
ls -la /dev/input/js0

# In container, test with:
ros2 run joy joy_node
ros2 topic echo /joy
```

## Roadmap

### Phase 1: Hardware Integration (Current)
- [ ] Complete ODrive hardware interface

### Phase 2: Motion Planning
- [ ] MoveIt2 configuration
- [ ] Collision geometry setup
- [ ] Path planning integration

### Phase 3: Advanced Features
- [ ] Vision system integration