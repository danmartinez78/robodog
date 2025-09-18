# ShadowHound Usage Guide

This document provides comprehensive instructions for installing, configuring, and operating the ShadowHound robotic platform.

## Table of Contents
1. [System Requirements](#system-requirements)
2. [Installation](#installation)
3. [Configuration](#configuration)
4. [Basic Operation](#basic-operation)
5. [Advanced Features](#advanced-features)
6. [Troubleshooting](#troubleshooting)

## System Requirements

### Supported Systems and ROS2 Distributions

| System | ROS2 Distribution | Build Status |
|--------|------------------|--------------|
| Ubuntu 22.04 | Iron | ✅ Tested |
| Ubuntu 22.04 | Humble | ✅ Tested |
| Ubuntu 22.04 | Rolling | ✅ Tested |

### Hardware Requirements
- **Computer**: Linux machine with ROS2 installed
- **Robot**: Unitree GO2 (AIR/PRO/EDU) with network connectivity
- **Optional**: Game controller (Xbox, PlayStation, Logitech F710, etc.)

## Installation

### 1. Create Workspace and Clone Repository

```bash
mkdir -p ros2_ws
cd ros2_ws
git clone --recurse-submodules https://github.com/your-username/shadowhound.git src
```

### 2. Install System Dependencies

```bash
# ROS2 packages
sudo apt install ros-$ROS_DISTRO-image-tools
sudo apt install ros-$ROS_DISTRO-vision-msgs

# System packages
sudo apt install python3-pip clang portaudio19-dev
```

### 3. Install Python Dependencies

```bash
cd src
pip install -r requirements.txt
cd ..
```

> **Important**: Pay attention to error messages during `pip install`. If it doesn't complete cleanly, various features may not work. For example, `open3d` doesn't yet support Python 3.12, so you may need to set up a Python 3.11 virtual environment first.

### 4. Build the Workspace

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

## Configuration

### Robot Network Setup

1. **Set up GO2 in Wi-Fi mode** using the mobile app
2. **Find the robot's IP address**:
   - Open the Unitree mobile app
   - Go to **Device → Data → Automatic Machine Inspection**
   - Look for **STA Network: wlan0** to find the IP address

### Environment Variables

```bash
# Required: Set robot IP address
export ROBOT_IP="192.168.1.100"  # Replace with your robot's IP

# Required: Set connection type
export CONN_TYPE="webrtc"  # or "cyclonedds"

# Optional: Multiple robots
export ROBOT_IP="192.168.1.100,192.168.1.101,192.168.1.102"

# Optional: 3D point cloud saving
export MAP_SAVE=True
export MAP_NAME="3d_map"
```

## Basic Operation

### 1. Launch the System

```bash
source install/setup.bash
export ROBOT_IP="your_robot_ip"
export CONN_TYPE="webrtc"
ros2 launch go2_robot_sdk robot.launch.py
```

### 2. System Components

The `robot.launch.py` starts the following components simultaneously:

- **robot_state_publisher**: Robot model and transforms
- **ros2_go2_video**: Front color camera stream
- **pointcloud_to_laserscan_node**: Convert point clouds to laser scans
- **go2_robot_sdk/go2_driver_node**: Main robot driver
- **lidar_processor/lidar_to_pointcloud**: LiDAR data processing
- **rviz2**: Visualization interface
- **joy**: Joystick/gamepad driver
- **teleop_twist_joy**: Joystick to velocity conversion
- **twist_mux**: Command multiplexer with prioritization
- **foxglove_launch**: Foxglove bridge for web visualization
- **slam_toolbox**: SLAM mapping
- **nav2**: Navigation stack

### 3. Initial System Behavior

When launched successfully:
- **RViz** opens with real-time robot visualization
- **LiDAR data** begins accumulating immediately
- **Camera stream** appears after ~4 seconds
- **Robot** waits for joystick commands
- **SLAM** starts building a map as the robot moves

## Advanced Features

### SLAM and Mapping

#### Creating Your First Map

1. **Mark a starting position**: Use painter's tape to create a 'dock' rectangle on the floor
2. **Initialize SLAM**: In RViz's SlamToolboxPlugin, click "Start At Dock"
3. **Explore the space**: Use your controller to manually drive through rooms
4. **Monitor progress**: Watch the map accumulate in RViz (white=free, black=occupied, grey=unknown)
5. **Save the map**:
   - Enter filename in "Save Map" field → click "Save Map"
   - Enter filename in "Serialize Map" field → click "Serialize Map"

This creates files in your workspace:
- `map_name.yaml`: Map metadata and image file path
- `map_name.pgm`: Map image with pixel data
- `map_name.data`: Additional map data
- `map_name.posegraph`: SLAM pose graph

#### Loading and Extending Maps

- Enter map filename (without extension) in "Deserialize Map" field
- Click "Deserialize Map" to load
- Continue mapping to extend the existing map

### Autonomous Navigation

#### Setup Navigation

1. **Load your map** using the SlamToolboxPlugin
2. **Verify robot position**: Ensure the robot is correctly oriented relative to the map
3. **Check Navigation status** in RViz Navigation 2 plugin:
   - Navigation: active
   - Localization: inactive → active (after setting initial pose)
   - Feedback: unknown → active

#### Setting Navigation Goals

1. **Set initial pose** (if needed): Use "2D Pose Estimate" in RViz
2. **Set navigation goal**: Use "Nav2 Goal" tool in RViz
   - Click to set target position
   - Drag to set final orientation (green arrow)

> **⚠️ Safety Warning**: 
> - Always supervise the robot during autonomous navigation
> - Be ready to pick up the robot if it behaves unexpectedly
> - Ensure the map is accurate and the robot's initial position is correct
> - Inaccurate maps can cause the robot to attempt driving through walls

### Object Detection and Tracking

#### Setup Object Detection

1. **Launch the main system** first
2. **Start object detection**:
   ```bash
   source install/setup.bash
   ros2 run coco_detector coco_detector_node
   ```

#### View Detection Results

**Text output**:
```bash
source install/setup.bash
ros2 topic echo /detected_objects
```

**Visual output**:
```bash
source install/setup.bash
ros2 run image_tools showimage --ros-args -r /image:=/annotated_image
```

#### Detection Configuration

```bash
# Example with custom parameters
ros2 run coco_detector coco_detector_node --ros-args \
  -p publish_annotated_image:=False \
  -p device:=cuda \
  -p detection_threshold:=0.7
```

Parameters:
- `publish_annotated_image`: Show bounding boxes (default: True)
- `device`: Processing device (default: cpu, options: cuda)
- `detection_threshold`: Confidence threshold 0.0-1.0 (default: 0.9)

### Docker Deployment

Set environment variables and run:

```bash
cd docker
ROBOT_IP=<ROBOT_IP> CONN_TYPE=<webrtc/cyclonedds> docker-compose up --build
```

### Connection Types

#### WebRTC (Wi-Fi)
```bash
export CONN_TYPE="webrtc"
```
- Uses Wi-Fi connection
- Close mobile app before connecting
- Default option for most use cases

#### CycloneDDS (Ethernet)
```bash
export CONN_TYPE="cyclonedds"
```
- Uses wired Ethernet connection
- More stable for development
- Lower latency

### Foxglove Studio Integration

#### Installation
```bash
sudo snap install foxglove-studio
```

#### Connection
1. Open Foxglove Studio
2. Click "Open Connection"
3. Select "Foxglove WebSocket"
4. Use default URL: `ws://localhost:8765`
5. Click "Open"

### WebRTC Command Interface

Send commands to the robot using the WebRTC topic:

```bash
# Basic command structure
ros2 topic pub /webrtc_req unitree_go/msg/WebRtcReq \
  "{api_id: <API_ID>, parameter: '<PARAMETER>', topic: '<TOPIC>', priority: <0|1>}" --once

# Example: Handshake command
ros2 topic pub /webrtc_req unitree_go/msg/WebRtcReq \
  "{api_id: 1016, topic: 'rt/api/sport/request'}" --once
```

### 3D Point Cloud Data Collection

Enable automatic point cloud saving:

```bash
export MAP_SAVE=True
export MAP_NAME="3d_map"
```

- Saves `.ply` files every 10 seconds
- Raw LiDAR data (not Nav2 maps)
- Useful for debugging and analysis

## Troubleshooting

### Joystick/Gamepad Issues

#### Logitech F710 Troubleshooting

1. **Check physical switch**: Set to DirectInput mode (D) on the back
2. **Verify detection**:
   ```bash
   ros2 run joy joy_enumerate_devices
   ```
3. **Test raw output**:
   ```bash
   ros2 run joy joy_node --ros-args -p device_id:=0
   ros2 topic echo /joy  # In another terminal
   ```

#### Common Gamepad Solutions

- **No device found**: Try different USB ports, check drivers
- **Wrong button mapping**: Verify button indices in go2_robot_sdk configuration
- **Permission issues**: Add user to input group or set udev rules

### WSL2 Joystick Setup

#### 1. Share Device with WSL2
Follow [Microsoft's USB device guide](https://learn.microsoft.com/en-us/windows/wsl/connect-usb)

#### 2. Build WSL2 Kernel with Joystick Drivers
Follow [usbipd-win guide](https://github.com/dorssel/usbipd-win/wiki/WSL-support#building-your-own-wsl-2-kernel-with-additional-drivers)

Update `.config` with values from [this GitHub issue](https://github.com/microsoft/WSL/issues/7747#issuecomment-1328217406)

#### 3. Set Device Permissions
Create `/etc/udev/rules.d/99-userdev-input.rules`:
```
KERNEL=="event*", SUBSYSTEM=="input", RUN+="/usr/bin/setfacl -m u:YOURUSERNAME:rw $env{DEVNAME}"
```

Run as root:
```bash
udevadm control --reload-rules && udevadm trigger
```

### Connection Issues

#### WebRTC Problems
- Close the Unitree mobile app before connecting
- Verify robot IP address is correct
- Check Wi-Fi network connectivity
- Ensure robot is in Wi-Fi mode

#### CycloneDDS Problems
- Verify Ethernet cable connection
- Check network configuration
- Ensure both devices are on same subnet

### Performance Issues

#### Low Frame Rates
- **LiDAR**: Should run at ~7Hz (improved from 2Hz)
- **Joint states**: Limited to 1Hz due to firmware v1.1.7
- **Camera**: Should appear within 4 seconds

#### Navigation Problems
- **Spinning/no motion**: Lower control loop rates in nav2_params.yaml
- **Wall collision**: Verify map accuracy and robot localization
- **Path planning failures**: Check for obstacles in path, adjust planner parameters

### RViz Display Issues

#### Too Much Information
- Deselect the `map` topic to see other data streams clearly
- Adjust topic display settings individually

#### Data Stream Verification
Monitor these topics for proper data flow:
- `/scan`: Laser scan data
- `/map`: SLAM-generated map
- `/camera/image`: Camera feed
- `/cloud`: Point cloud data
- `/odom`: Odometry

### Build Errors

#### Dependency Issues
```bash
# Update package lists
sudo apt update

# Install missing dependencies
rosdep install --from-paths src --ignore-src -r -y

# Clean and rebuild
rm -rf build install log
colcon build
```

#### Python Package Issues
```bash
# Create virtual environment for Python 3.11
python3.11 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

## Support and Community

For additional support:
1. Check the [project documentation](../docs/)
2. Review [common issues](https://github.com/your-username/shadowhound/issues)
3. Consult the [original SDK documentation](original_readme_archived.md)
4. Join the robotics community discussions

---

**Remember**: Always prioritize safety when operating the robot. Start with supervised operation and gradually increase autonomy as you become familiar with the system's behavior.
