# ShadowHound Mission Agent - Design Documentation

**Package**: `shadowhound_mission_agent`  
**Version**: 0.1.0  
**Last Updated**: October 4, 2025

---

## Table of Contents

1. [Overview](#overview)
2. [Architecture](#architecture)
3. [Components](#components)
4. [API Reference](#api-reference)
5. [Configuration](#configuration)
6. [Usage Examples](#usage-examples)
7. [Development Notes](#development-notes)
8. [Future Enhancements](#future-enhancements)

---

## Overview

### Purpose

The ShadowHound Mission Agent is a ROS2 node that bridges natural language mission commands with autonomous robot execution. It serves as the integration layer between ROS2's pub/sub messaging and DIMOS's agent/skill framework.

### Design Philosophy

1. **Thin Integration Layer**: Minimal custom code, maximum DIMOS leverage
2. **ROS2 Native**: Standard topics, parameters, and lifecycle
3. **Flexible Backend**: Support cloud and local LLM backends
4. **Mock-First Development**: Can develop and test without hardware
5. **Safety-First**: Controlled execution with status reporting

### Key Features

- ✅ Natural language command processing via ROS2 topics
- ✅ DIMOS agent integration (OpenAI, Planning)
- ✅ 40+ robot skills accessible
- ✅ Mock robot support for development
- ✅ Configurable agent backends
- ✅ Real-time status publishing
- ✅ Graceful error handling

---

## Architecture

### System Context

```
┌─────────────────────────────────────────────────────────────────┐
│                        External Systems                          │
│  - User Interface (CLI, GUI, mobile app)                        │
│  - Mission Planners                                              │
│  - Monitoring Dashboards                                         │
└────────────────────┬────────────────────────────────────────────┘
                     │ ROS2 Topics
                     │
┌────────────────────▼────────────────────────────────────────────┐
│              ShadowHound Mission Agent                          │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │  Mission Agent Node (ROS2)                               │  │
│  │  - Topic subscription/publishing                         │  │
│  │  - Parameter management                                  │  │
│  │  - Lifecycle management                                  │  │
│  └────────────┬─────────────────────────────────────────────┘  │
│               │                                                  │
│  ┌────────────▼─────────────────────────────────────────────┐  │
│  │  Agent Coordinator                                        │  │
│  │  - Command routing                                        │  │
│  │  - Agent selection (OpenAI vs Planning)                  │  │
│  │  - Error handling and recovery                           │  │
│  └────────────┬─────────────────────────────────────────────┘  │
└───────────────┼──────────────────────────────────────────────────┘
                │
┌───────────────▼──────────────────────────────────────────────────┐
│                      DIMOS Framework                              │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │  Agent Layer                                              │   │
│  │  - OpenAIAgent: Direct command processing                │   │
│  │  - PlanningAgent: Multi-step mission planning            │   │
│  │  - Semantic Memory (ChromaDB)                            │   │
│  │  - Tool/Function calling                                 │   │
│  └────────────┬─────────────────────────────────────────────┘   │
│               │                                                   │
│  ┌────────────▼─────────────────────────────────────────────┐   │
│  │  Skills Layer                                             │   │
│  │  - UnitreeSkills (40+ robot behaviors)                   │   │
│  │  - Skill registry and execution                          │   │
│  │  - Safety validation                                     │   │
│  └────────────┬─────────────────────────────────────────────┘   │
│               │                                                   │
│  ┌────────────▼─────────────────────────────────────────────┐   │
│  │  Robot Interface                                          │   │
│  │  - UnitreeGo2: Robot abstraction                         │   │
│  │  - UnitreeROSControl: ROS2 bridge                        │   │
│  │  - Mock support for testing                              │   │
│  └───────────────────────────────────────────────────────────┘   │
└───────────────────────────────────────────────────────────────────┘
```

### Data Flow

```
User Command → /mission_command topic
                      ↓
              Mission Agent Node
                      ↓
              Command Validation
                      ↓
              Agent Selection
             /                \
    OpenAIAgent            PlanningAgent
            \                /
             ↓              ↓
          DIMOS Skills (40+)
                 ↓
          Robot Execution
                 ↓
      /mission_status topic → User Feedback
```

---

## Components

### 1. MissionAgentNode (ROS2 Layer)

**File**: `mission_agent.py`  
**Class**: `MissionAgentNode(Node)`

#### Responsibilities
- ROS2 node lifecycle management
- Topic subscription/publishing
- Parameter declaration and retrieval
- DIMOS availability checking
- Component initialization and cleanup

#### Initialization Sequence

```python
1. super().__init__("shadowhound_mission_agent")
   ├─ Create ROS2 node
   
2. Declare parameters
   ├─ agent_backend: "cloud" | "local"
   ├─ mock_robot: true | false
   └─ use_planning_agent: true | false
   
3. Check DIMOS availability
   ├─ Validate imports
   └─ Raise error if not available
   
4. Initialize robot interface
   ├─ Create UnitreeROSControl(mock_connection=mock_robot)
   ├─ Create UnitreeGo2(ros_control, disable_video_stream=True)
   └─ Log initialization status
   
5. Initialize skill library
   ├─ Create UnitreeSkills(robot=self.robot)
   └─ Log skill count
   
6. Initialize agent
   ├─ Select agent type (OpenAI vs Planning)
   ├─ Configure with robot and skills
   └─ Log agent type
   
7. Create ROS2 interfaces
   ├─ Subscribe to /mission_command
   └─ Publish to /mission_status
   
8. Ready to receive commands
```

#### Topics

**Subscribed**:
- `/mission_command` (std_msgs/String)
  - Natural language mission commands
  - Example: "stand up and wave hello"
  - Single-shot or streaming commands

**Published**:
- `/mission_status` (std_msgs/String)
  - Mission execution status
  - Format: "STATE: command | Result: details"
  - States: EXECUTING, COMPLETED, FAILED

#### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `agent_backend` | string | "cloud" | Agent backend: "cloud" (OpenAI) or "local" |
| `mock_robot` | bool | true | Use mock robot connection for testing |
| `use_planning_agent` | bool | false | Enable planning agent for complex missions |

---

### 2. Agent Coordinator

**Responsibility**: Route commands to appropriate DIMOS agent

#### Agent Selection Logic

```python
if use_planning_agent:
    agent = PlanningAgent(robot, skills)
    # Multi-step mission planning
    # Decomposes complex commands
    # Sequential execution with validation
else:
    agent = OpenAIAgent(robot, skills)
    # Direct command processing
    # Single-step or simple missions
    # Faster response time
```

#### Command Processing

**OpenAI Agent Mode**:
```python
command = "stand up and wave"
↓
agent.process_text(command)
↓
LLM interprets → Selects skills → Executes → Returns result
```

**Planning Agent Mode**:
```python
command = "patrol the room"
↓
agent.plan_and_execute(command)
↓
LLM creates plan → Validates steps → Executes sequence → Returns result
```

---

### 3. Callback Handler

**Method**: `mission_callback(self, msg: String)`

#### Flow

```python
1. Receive command
   ├─ Extract msg.data
   └─ Log command
   
2. Publish EXECUTING status
   ├─ Format: "EXECUTING: {command}"
   └─ Notify subscribers
   
3. Execute via agent
   ├─ If use_planning: agent.plan_and_execute(command)
   ├─ Else: agent.process_text(command)
   └─ Capture result
   
4. Handle result
   ├─ Success: Publish "COMPLETED: {command} | Result: {result}"
   ├─ Failure: Publish "FAILED: {command} | Error: {error}"
   └─ Log outcome
   
5. Exception handling
   ├─ Catch all exceptions
   ├─ Log error details
   └─ Publish FAILED status
```

---

### 4. Lifecycle Management

#### Startup
```python
def __init__(self):
    # Initialize all components
    # Register topics and parameters
    # Ready to receive commands
```

#### Shutdown
```python
def destroy_node(self):
    # Dispose agent (cleanup memory, subscriptions)
    # Cleanup robot interface
    # Call parent destroy_node()
```

#### Main Entry Point
```python
def main(args=None):
    rclpy.init(args=args)
    try:
        node = MissionAgentNode()
        rclpy.spin(node)  # Process callbacks
    except KeyboardInterrupt:
        pass  # Graceful Ctrl+C
    except Exception as e:
        print(f"Fatal error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()
```

---

## API Reference

### ROS2 Interface

#### Topics

##### Input: `/mission_command`
```yaml
Type: std_msgs/String
Rate: Variable (command-driven)
Description: Natural language mission commands

Examples:
  - "stand up"
  - "walk forward 5 meters"
  - "perform dance 1"
  - "patrol the area and return to start"
```

##### Output: `/mission_status`
```yaml
Type: std_msgs/String
Rate: Per command execution
Description: Mission execution status and results

Format:
  EXECUTING: <command>
  COMPLETED: <command> | Result: <result_data>
  FAILED: <command> | Error: <error_message>

Examples:
  - "EXECUTING: stand up"
  - "COMPLETED: stand up | Result: Robot standing"
  - "FAILED: walk forward | Error: Obstacle detected"
```

#### Parameters

```yaml
agent_backend:
  type: string
  default: "cloud"
  values: ["cloud", "local"]
  description: LLM backend selection
  runtime_changeable: false

mock_robot:
  type: bool
  default: true
  description: Use mock robot for testing
  runtime_changeable: false

use_planning_agent:
  type: bool
  default: false
  description: Enable planning agent for complex missions
  runtime_changeable: false
```

### Python API (Internal)

#### Class: MissionAgentNode

```python
class MissionAgentNode(Node):
    """ROS2 node for autonomous mission execution."""
    
    def __init__(self):
        """Initialize node, robot, skills, and agent."""
        
    def _init_agent(self):
        """Initialize DIMOS agent based on configuration."""
        
    def mission_callback(self, msg: String):
        """Handle incoming mission commands."""
        
    def destroy_node(self):
        """Clean up resources."""
```

---

## Configuration

### Launch File Parameters

**File**: `launch/mission_agent.launch.py`

```python
# Basic launch (mock robot, cloud agent)
ros2 launch shadowhound_mission_agent mission_agent.launch.py

# Real robot
ros2 launch shadowhound_mission_agent mission_agent.launch.py \
    mock_robot:=false

# Planning agent
ros2 launch shadowhound_mission_agent mission_agent.launch.py \
    use_planning_agent:=true

# Local LLM backend
ros2 launch shadowhound_mission_agent mission_agent.launch.py \
    agent_backend:=local
```

### Environment Variables

```bash
# Required for cloud agent
export OPENAI_API_KEY="sk-..."

# Optional ROS configuration
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Robot connection (real robot)
export GO2_IP="192.168.12.1"

# DIMOS path (if not installed)
export PYTHONPATH="${PWD}/src/dimos-unitree:${PYTHONPATH}"
```

---

## Usage Examples

### Example 1: Basic Command

```bash
# Terminal 1: Launch agent
ros2 launch shadowhound_mission_agent mission_agent.launch.py

# Terminal 2: Send command
ros2 topic pub --once /mission_command std_msgs/String \
    "data: 'stand up'"

# Terminal 3: Monitor status
ros2 topic echo /mission_status
```

**Expected Output**:
```
data: EXECUTING: stand up
---
data: COMPLETED: stand up | Result: Robot standing
---
```

### Example 2: Complex Mission with Planning Agent

```bash
# Launch with planning agent
ros2 launch shadowhound_mission_agent mission_agent.launch.py \
    use_planning_agent:=true

# Send complex command
ros2 topic pub --once /mission_command std_msgs/String \
    "data: 'patrol the room by walking forward 5 meters, turning around, and returning to the start'"
```

**Agent Behavior**:
1. Decomposes into steps: walk forward → turn → walk back
2. Validates each step
3. Executes sequentially
4. Reports completion

### Example 3: Python Script

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MissionCommander(Node):
    def __init__(self):
        super().__init__('mission_commander')
        
        self.cmd_pub = self.create_publisher(
            String, '/mission_command', 10
        )
        
        self.status_sub = self.create_subscription(
            String, '/mission_status',
            self.status_callback, 10
        )
    
    def send_mission(self, command):
        msg = String()
        msg.data = command
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'Sent: {command}')
    
    def status_callback(self, msg):
        self.get_logger().info(f'Status: {msg.data}')

def main():
    rclpy.init()
    node = MissionCommander()
    
    # Send commands
    node.send_mission('stand up')
    rclpy.spin_once(node, timeout_sec=5)
    
    node.send_mission('wave hello')
    rclpy.spin_once(node, timeout_sec=5)
    
    node.send_mission('sit down')
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

---

## Development Notes

### Current Implementation Status

#### ✅ Completed
- [x] ROS2 node infrastructure
- [x] DIMOS agent integration
- [x] OpenAI agent support
- [x] Planning agent support
- [x] Mock robot mode
- [x] Basic error handling
- [x] Status publishing
- [x] Launch file configuration

#### 🔄 In Progress
- [ ] Hardware testing with real Go2
- [ ] Advanced error recovery
- [ ] Mission queue management
- [ ] Progress reporting for long missions

#### 📋 Planned
- [ ] Mission templates/presets
- [ ] Video stream integration
- [ ] Custom skill registration
- [ ] Multi-robot coordination

### Design Decisions

#### Why Thin Integration Layer?
- **Maintainability**: Less custom code = fewer bugs
- **Flexibility**: Easy to adapt as DIMOS evolves
- **Leverage Expertise**: DIMOS team's work is battle-tested
- **Focus**: Concentrate on ROS2 integration, not reinventing agents

#### Why String Messages?
- **Simplicity**: Natural language commands are strings
- **Flexibility**: No rigid message structure needed
- **Debugging**: Easy to inspect and test with CLI
- **Future**: Can add structured messages later if needed

#### Why Mock-First?
- **Development Speed**: Don't need hardware to build features
- **Testing**: Automated tests without robot
- **Safety**: Validate logic before hardware
- **CI/CD**: Run tests in containers

### Testing Strategy

#### Unit Tests
```bash
colcon test --packages-select shadowhound_mission_agent
```

#### Integration Tests (Mock Robot)
```bash
# Launch agent
ros2 launch shadowhound_mission_agent mission_agent.launch.py

# Run test script
python3 test/integration_test.py
```

#### Hardware Tests (Real Robot)
```bash
# Connect to robot
ros2 launch shadowhound_mission_agent mission_agent.launch.py \
    mock_robot:=false

# Manual test sequence
ros2 topic pub --once /mission_command std_msgs/String "data: 'stand up'"
# Verify robot stands
```

---

## Future Enhancements

### Short-term (v0.2.0)

1. **Mission Queue**
   ```python
   # Accept multiple commands
   # Execute in sequence
   # Report progress for each
   ```

2. **Progress Updates**
   ```python
   # Long missions report intermediate status
   # Example: "Step 2 of 5 complete"
   ```

3. **Mission Cancellation**
   ```python
   # New topic: /mission_cancel
   # Stop current mission safely
   ```

4. **Parameter Reconfiguration**
   ```python
   # Change agent backend at runtime
   # Switch mock/real mode dynamically
   ```

### Mid-term (v0.3.0)

1. **Vision Integration**
   ```python
   # Process camera feed
   # Visual navigation commands
   # Object detection integration
   ```

2. **Custom Skills**
   ```python
   # Register ShadowHound-specific skills
   # Mission-specific behaviors
   # Extended skill library
   ```

3. **Mission Templates**
   ```yaml
   # Predefined mission sequences
   # Parameterized templates
   # Library of common missions
   ```

4. **Telemetry and Logging**
   ```python
   # Detailed execution logs
   # Performance metrics
   # Diagnostic data
   ```

### Long-term (v1.0.0)

1. **Multi-Robot Coordination**
   ```python
   # Command multiple robots
   # Coordinated missions
   # Swarm behaviors
   ```

2. **Learning and Adaptation**
   ```python
   # Learn from execution
   # Improve over time
   # User preference learning
   ```

3. **Advanced Planning**
   ```python
   # Path planning integration
   # Obstacle avoidance
   # Dynamic replanning
   ```

4. **Web Interface**
   ```python
   # Mission dashboard
   # Real-time visualization
   # Remote control
   ```

---

## Contributing

### Code Style
- Follow ROS2 Python style guide
- Use type hints
- Document public APIs
- Include docstrings (Google style)

### Pull Request Process
1. Create feature branch
2. Implement with tests
3. Update documentation
4. Submit PR with description

### Testing Requirements
- Unit tests for new functions
- Integration tests for workflows
- Hardware validation before merge

---

## References

- **DIMOS Framework**: https://github.com/dimensionalOS/dimos-unitree
- **ROS2 Humble**: https://docs.ros.org/en/humble/
- **Unitree Go2**: https://www.unitree.com/go2
- **ShadowHound Docs**: `/workspaces/shadowhound/docs/`

---

**Last Updated**: October 4, 2025  
**Maintainer**: ShadowHound Team  
**License**: Apache 2.0
