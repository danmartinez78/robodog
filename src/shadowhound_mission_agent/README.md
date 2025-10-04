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

### Standalone Web Interface Testing

The web interface can be tested independently:

```bash
cd src/shadowhound_mission_agent/shadowhound_mission_agent
python3 web_interface.py

# Open http://localhost:8080
# Commands will be printed to console
```

### Reusing Web Interface in Other Projects

```python
from shadowhound_mission_agent.web_interface import WebInterface

def my_command_handler(command: str) -> dict:
    """Your custom command processing."""
    print(f"Received: {command}")
    return {"success": True, "message": f"Executed {command}"}

# Create and start
web = WebInterface(
    command_callback=my_command_handler,
    port=8080
)
web.start()

# Broadcast updates to clients
web.broadcast_sync("Task completed!")
```

## Configuration

### Environment Variables

- **`OPENAI_API_KEY`** - Required for cloud backend with OpenAI
- **`ROS_DOMAIN_ID`** - ROS2 network domain (default: 0)

## Troubleshooting

### Web Interface Won't Start

```bash
# Check if port is already in use
sudo lsof -i :8080

# Use different port
ros2 launch shadowhound_mission_agent bringup.launch.py web_port:=9000
```

### DIMOS Import Errors

```bash
# Ensure DIMOS is in PYTHONPATH
export PYTHONPATH=/workspaces/shadowhound/src/dimos-unitree:$PYTHONPATH

# Or source workspace
source /workspaces/shadowhound/install/setup.bash
```

### Mission Commands Not Executing

1. Check agent logs: `ros2 node info /shadowhound_mission_agent`
2. Verify OPENAI_API_KEY is set
3. Check robot connection (if using real hardware)
4. Try with mock_robot:=true to isolate issues

## DIMOS Integration

This node acts as a thin ROS2 + Web wrapper around DIMOS agents:

### Running Tests

```bash
colcon test --packages-select shadowhound_mission_agent
```

### Code Style

This package follows ROS2 Python style guidelines.

## License

Apache 2.0
