# ShadowHound Mission Agent

ROS2 package that provides autonomous mission execution for the Unitree Go2 robot using the DIMOS framework.

## Overview

This package integrates the DIMOS agent system with ROS2, enabling natural language mission commands to be translated into robot actions through the DIMOS skills library.

## Features

- **DIMOS Integration**: Leverages OpenAIAgent and PlanningAgent from DIMOS
- **Skills Access**: Direct access to 40+ Unitree Go2 skills
- **Mock Mode**: Can run without hardware using mock robot connection
- **Flexible Agent Backend**: Supports cloud (OpenAI) and local LLM options

## Dependencies

- ROS2 Humble
- DIMOS framework (`dimos-unitree`)
- OpenAI API key (for cloud agent)

## Nodes

### mission_agent

Main mission execution node.

**Subscribed Topics:**
- `mission_command` (std_msgs/String): Natural language mission commands

**Published Topics:**
- `mission_status` (std_msgs/String): Mission execution status and results

**Parameters:**
- `agent_backend` (string, default: "cloud"): Agent backend type
- `mock_robot` (bool, default: true): Use mock robot connection
- `use_planning_agent` (bool, default: false): Use planning agent for complex missions

## Usage

### Launch with Mock Robot

```bash
ros2 launch shadowhound_mission_agent mission_agent.launch.py
```

### Launch with Real Robot

```bash
ros2 launch shadowhound_mission_agent mission_agent.launch.py mock_robot:=false
```

### Send Mission Command

```bash
ros2 topic pub /mission_command std_msgs/String "data: 'Stand up and wave hello'"
```

### Monitor Status

```bash
ros2 topic echo /mission_status
```

## Configuration

### Environment Variables

- `OPENAI_API_KEY`: Required for cloud agent backend
- `ROS_DOMAIN_ID`: ROS2 domain ID (default: 42)

### Planning Agent Mode

For multi-step missions, enable planning agent:

```bash
ros2 launch shadowhound_mission_agent mission_agent.launch.py use_planning_agent:=true
```

## Examples

### Simple Commands

```bash
# Stand up
ros2 topic pub /mission_command std_msgs/String "data: 'stand up'"

# Dance
ros2 topic pub /mission_command std_msgs/String "data: 'perform dance 1'"

# Sit down
ros2 topic pub /mission_command std_msgs/String "data: 'sit down'"
```

### Complex Missions (with Planning Agent)

```bash
ros2 topic pub /mission_command std_msgs/String "data: 'Patrol the area by walking forward 5 meters, turning around, and returning to start'"
```

## Development

### Running Tests

```bash
colcon test --packages-select shadowhound_mission_agent
```

### Code Style

This package follows ROS2 Python style guidelines.

## License

Apache 2.0
