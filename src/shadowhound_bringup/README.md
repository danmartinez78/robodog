# ShadowHound Bringup

Launch files and configurations for the ShadowHound autonomous robot system.

## Overview

This package provides the main entry points to start the complete ShadowHound system, integrating DIMOS and ShadowHound components.

## Launch Files

### shadowhound.launch.py

Main launch file that brings up the complete system.

**Arguments:**
- `mock_robot` (bool, default: true): Use mock robot connection
- `agent_backend` (string, default: "cloud"): Agent backend type (cloud/local)
- `use_planning_agent` (bool, default: false): Enable planning agent for complex missions

## Usage

### Basic Launch (Mock Robot)

```bash
ros2 launch shadowhound_bringup shadowhound.launch.py
```

### Launch with Real Robot

```bash
ros2 launch shadowhound_bringup shadowhound.launch.py mock_robot:=false
```

### Launch with Planning Agent

```bash
ros2 launch shadowhound_bringup shadowhound.launch.py use_planning_agent:=true
```

### Full Configuration Example

```bash
ros2 launch shadowhound_bringup shadowhound.launch.py \
    mock_robot:=false \
    agent_backend:=cloud \
    use_planning_agent:=true
```

## Configuration Files

### default.yaml

Default parameters for the mission agent. Located in `config/default.yaml`.

Customize by copying and modifying:

```bash
cp config/default.yaml config/my_config.yaml
# Edit my_config.yaml
```

## Environment Setup

### Required Environment Variables

```bash
export OPENAI_API_KEY="sk-..."  # For cloud agent backend
export ROS_DOMAIN_ID=42          # Isolated ROS network
```

### Optional Variables

```bash
export GO2_IP="192.168.1.103"   # Robot IP (for real robot)
```

## System Requirements

- ROS2 Humble
- DIMOS framework
- shadowhound_mission_agent package
- Python 3.10+

## Quick Start

1. **Source workspace:**
   ```bash
   source install/setup.bash
   ```

2. **Set OpenAI API key:**
   ```bash
   export OPENAI_API_KEY="your-key-here"
   ```

3. **Launch system:**
   ```bash
   ros2 launch shadowhound_bringup shadowhound.launch.py
   ```

4. **Send mission command:**
   ```bash
   ros2 topic pub /mission_command std_msgs/String "data: 'stand up and hello'"
   ```

5. **Monitor status:**
   ```bash
   ros2 topic echo /mission_status
   ```

## Debugging

### Check Node Status

```bash
ros2 node list
ros2 node info /mission_agent
```

### View Topics

```bash
ros2 topic list
ros2 topic info /mission_command
```

### Check Parameters

```bash
ros2 param list /mission_agent
ros2 param get /mission_agent agent_backend
```

## License

Apache 2.0
