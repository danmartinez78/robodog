# Mission Agent Architecture

## Overview

The ShadowHound mission agent provides a clean, layered architecture that separates ROS concerns from business logic. This enables testing without ROS, reuse in non-ROS contexts (scripts, notebooks, web apps), and follows DIMOS framework conventions.

## Architectural Decision

**Key Insight**: DIMOS already provides the agent abstraction we need. Skills are the extension point, not agent types.

After analysis (see `docs/AGENT_REFACTOR_ANALYSIS.md`), we simplified from a custom agent wrapper to using DIMOS agents directly, with proper separation of concerns.

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│                    MissionAgentNode (ROS Wrapper)                   │
│                                                                      │
│  ┌────────────────────────────────────────────────────────────────┐ │
│  │  ROS-Specific Concerns:                                        │ │
│  │  • Node lifecycle (init, spin, shutdown)                       │ │
│  │  • Parameters (agent_backend, robot_ip, etc.)                  │ │
│  │  • Topic subscriptions (/mission_command)                      │ │
│  │  • Topic publishers (/mission_status)                          │ │
│  │  • ROS logging bridge                                          │ │
│  │  • Web interface management                                    │ │
│  └────────────────────────────────────────────────────────────────┘ │
│                              │                                       │
│                              │ delegates to                          │
│                              ▼                                       │
│  ┌────────────────────────────────────────────────────────────────┐ │
│  │              MissionExecutor (Pure Python)                     │ │
│  │                                                                 │ │
│  │  Business Logic:                                                │ │
│  │  • Robot initialization (DIMOS UnitreeGo2)                      │ │
│  │  • Skill library setup (DIMOS MyUnitreeSkills)                  │ │
│  │  • Agent initialization (DIMOS OpenAIAgent/PlanningAgent)       │ │
│  │  • Mission execution logic                                      │ │
│  │  • Status queries                                               │ │
│  │                                                                 │ │
│  │  ✅ No ROS dependencies                                         │ │
│  │  ✅ Testable without ROS                                        │ │
│  │  ✅ Reusable in scripts, notebooks, web apps                    │ │
│  └────────────────────────────────────────────────────────────────┘ │
│                              │                                       │
│                              │ uses                                  │
│                              ▼                                       │
│  ┌────────────────────────────────────────────────────────────────┐ │
│  │                      DIMOS Framework                            │ │
│  │                                                                 │ │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐         │ │
│  │  │ OpenAIAgent  │  │PlanningAgent │  │  UnitreeGo2  │         │ │
│  │  │  (GPT-4)     │  │ (ReAct loop) │  │   (Robot)    │         │ │
│  │  └──────────────┘  └──────────────┘  └──────────────┘         │ │
│  │                                                                 │ │
│  │  ┌──────────────────────────────────────────────────┐          │ │
│  │  │           MyUnitreeSkills                        │          │ │
│  │  │  • move_forward, rotate, sit, stand              │          │ │
│  │  │  • Skills available via function calling         │          │ │
│  │  └──────────────────────────────────────────────────┘          │ │
│  └────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────┘
```

## Core Components

### 1. MissionAgentNode (ROS Wrapper)

**File**: `mission_agent.py` (292 lines)

**Purpose**: Thin ROS wrapper that handles ROS-specific concerns only.

**Responsibilities**:
- ROS node lifecycle management
- ROS parameter handling
- Topic subscriptions and publications
- Web interface integration
- ROS logging bridge

**Example**:
```python
# In a launch file
from launch_ros.actions import Node

Node(
    package='shadowhound_mission_agent',
    executable='mission_agent',
    parameters=[{
        'agent_backend': 'cloud',
        'robot_ip': '192.168.1.103',
        'agent_model': 'gpt-4-turbo',
    }]
)
```

**ROS Topics**:
- **Subscribed**: `/mission_command` (std_msgs/String) - Incoming mission commands
- **Published**: `/mission_status` (std_msgs/String) - Mission status updates

### 2. MissionExecutor (Pure Python Business Logic)

**File**: `mission_executor.py` (312 lines)

**Purpose**: Encapsulates all mission execution logic without ROS dependencies.

**Responsibilities**:
- Robot initialization (DIMOS `UnitreeGo2`)
- Skill library setup (DIMOS `MyUnitreeSkills`)
- Agent initialization (DIMOS `OpenAIAgent` or `PlanningAgent`)
- Mission execution
- Status queries

**Example Usage**:

```python
from shadowhound_mission_agent.mission_executor import (
    MissionExecutor,
    MissionExecutorConfig
)

# In a ROS node
config = MissionExecutorConfig(
    agent_backend="cloud",
    robot_ip="192.168.1.103",
    agent_model="gpt-4-turbo"
)
executor = MissionExecutor(config, logger=node.get_logger())
executor.initialize()
result = executor.execute_mission("patrol the perimeter")

# In a script (no ROS!)
import logging
logger = logging.getLogger(__name__)
config = MissionExecutorConfig()
executor = MissionExecutor(config, logger=logger)
executor.initialize()
result = executor.execute_mission("go to waypoint A")

# In a Jupyter notebook
executor = MissionExecutor(MissionExecutorConfig())
executor.initialize()
status = executor.get_robot_status()
skills = executor.get_available_skills()
result = executor.execute_mission("rotate 90 degrees")
```

**API**:
- `initialize()` - Initialize robot, skills, and agent
- `execute_mission(command: str) -> str` - Execute mission, return response
- `get_robot_status() -> Dict[str, Any]` - Get robot status info
- `get_available_skills() -> list` - Get list of available skills
- `cleanup()` - Clean up resources

### 3. MissionExecutorConfig (Configuration)

**Purpose**: Configuration dataclass for MissionExecutor.

```python
from shadowhound_mission_agent.mission_executor import MissionExecutorConfig

config = MissionExecutorConfig(
    agent_backend="cloud",          # 'cloud' or 'local'
    use_planning_agent=False,       # Use PlanningAgent vs OpenAIAgent
    robot_ip="192.168.1.103",       # Robot IP address
    webrtc_api_topic="webrtc_req",  # ROS topic for WebRTC commands
    agent_model="gpt-4-turbo"       # LLM model to use
)
```

## DIMOS Integration

We use DIMOS components directly (no wrapper):

### Agent Types

**OpenAIAgent** (default):
- Uses OpenAI function calling
- Best for general missions
- Configurable model (gpt-4-turbo, gpt-4o, etc.)

**PlanningAgent**:
- Uses ReAct-style planning loop
- Best for complex multi-step missions
- Built-in retry and error handling

**Selection**:
```python
# OpenAIAgent (default)
config = MissionExecutorConfig(use_planning_agent=False)

# PlanningAgent
config = MissionExecutorConfig(use_planning_agent=True)
```

### Skills Extension

**DIMOS Pattern**: Skills are the extension point, not agent types.

To add new capabilities:
1. Add skills to `MyUnitreeSkills` (in DIMOS)
2. Skills automatically available to all agents via function calling
3. No need to modify agent code

**Example** (in DIMOS):
```python
class MyUnitreeSkills(Skills):
    def __init__(self, robot):
        super().__init__(robot)
    
    @skill(description="Take a photo with the robot's camera")
    def capture_image(self) -> np.ndarray:
        """Capture an image from robot camera."""
        return self.robot.get_camera_image()
```

## Testing

### Unit Tests (Fast, No ROS)

Test `MissionExecutor` without ROS infrastructure:

```python
# test/test_mission_executor.py
from shadowhound_mission_agent.mission_executor import MissionExecutor

def test_config_defaults():
    config = MissionExecutorConfig()
    assert config.agent_backend == "cloud"
    assert config.robot_ip == "192.168.1.103"

def test_execute_mission_not_initialized():
    executor = MissionExecutor(MissionExecutorConfig())
    with pytest.raises(RuntimeError):
        executor.execute_mission("test")
```

**Run**: `pytest src/shadowhound_mission_agent/test/test_mission_executor.py`

**Results**: 10 passed, 4 skipped in 0.09s ⚡

### Integration Tests (Requires DIMOS)

Test with actual DIMOS framework:

```python
@pytest.mark.skipif(not DIMOS_AVAILABLE, reason="Requires DIMOS")
def test_full_mission_execution():
    config = MissionExecutorConfig()
    executor = MissionExecutor(config)
    executor.initialize()
    result = executor.execute_mission("sit down")
    assert "sit" in result.lower()
    executor.cleanup()
```

## Benefits of This Architecture

### ✅ Separation of Concerns

**ROS Concerns** (MissionAgentNode):
- Node lifecycle
- Parameters
- Topics
- Logging

**Business Logic** (MissionExecutor):
- Robot control
- Mission execution
- Status queries

### ✅ Testability

**Before**: Tests required full ROS stack
- Slow integration tests only
- Hard to mock
- Fragile

**After**: Unit tests without ROS
- Fast (0.09s)
- Easy to mock
- Reliable

### ✅ Reusability

**Use MissionExecutor in**:
- ROS nodes (MissionAgentNode)
- Jupyter notebooks (interactive development)
- CLI scripts (batch operations)
- Web applications (REST API)
- Test scripts (validation)

### ✅ Follows DIMOS Conventions

**Key Insight**: DIMOS already provides:
- Agent abstraction (`OpenAIAgent`, `PlanningAgent`)
- Skill system (`MyUnitreeSkills`)
- Robot interface (`UnitreeGo2`)

**Our Role**: Use DIMOS components, extend via skills.

**Not**: Create wrapper layers that duplicate DIMOS functionality.

## Migration from Old Architecture

If you have code using the old agent wrapper:

### Before (Old Wrapper)
```python
from shadowhound_mission_agent.agent import AgentFactory, BaseAgent

factory = AgentFactory(skills, robot, config)
agent = factory.create("openai")
result = agent.execute_mission("patrol")
# Returns MissionResult dataclass
```

### After (DIMOS Direct)
```python
from shadowhound_mission_agent.mission_executor import (
    MissionExecutor,
    MissionExecutorConfig
)

config = MissionExecutorConfig(agent_backend="cloud")
executor = MissionExecutor(config)
executor.initialize()
result = executor.execute_mission("patrol")
# Returns string response
```

**Why**: The old wrapper was unnecessary - DIMOS already provides agent abstraction. See `docs/AGENT_REFACTOR_ANALYSIS.md` for full rationale.

## Adding Vision Capabilities

For vision integration strategy, see `docs/VISION_INTEGRATION_DESIGN.md`.

**TL;DR**: Add vision as DIMOS skills (not agent types).

```python
class MyUnitreeSkills(Skills):
    @skill(description="Analyze an image with VLM")
    def analyze_scene(self) -> str:
        image = self.robot.get_camera_image()
        # Call VLM backend (GPT-4 Vision, Claude 3, etc.)
        return vlm_client.analyze(image, "What do you see?")
```

## Configuration

### Environment Variables

```bash
# Connection type (webrtc for high-level API, cyclonedds for low-level)
export CONN_TYPE=webrtc

# Robot IP address
export GO2_IP=192.168.1.103

# OpenAI API key (for cloud backend)
export OPENAI_API_KEY=sk-...
```

### ROS Parameters

```yaml
# config/mission_agent.yaml
shadowhound_mission_agent:
  ros__parameters:
    agent_backend: "cloud"        # 'cloud' or 'local'
    use_planning_agent: false     # Use PlanningAgent vs OpenAIAgent
    robot_ip: "192.168.1.103"     # Robot IP
    agent_model: "gpt-4-turbo"    # LLM model
    enable_web_interface: true    # Enable web dashboard
    web_port: 8080                # Web interface port
```

## Troubleshooting

### "DIMOS not available"

**Error**: `RuntimeError: DIMOS framework not available`

**Solution**: Ensure `dimos-unitree` is in `PYTHONPATH`:
```bash
source install/setup.bash  # After colcon build
```

### "Robot not initialized"

**Error**: `RuntimeError: Robot not initialized`

**Cause**: Called method before `executor.initialize()`

**Solution**:
```python
executor = MissionExecutor(config)
executor.initialize()  # Must call before execute_mission()
executor.execute_mission("patrol")
```

### "Skills not initialized"

**Error**: `RuntimeError: Skills not initialized`

**Cause**: Called method before initialization

**Solution**: Same as above - call `executor.initialize()` first.

## Future Enhancements

### Planned Features

1. **Vision Skills** - Add VLM-powered perception skills
2. **Async Execution** - Support for async mission execution
3. **Mission Plans** - Structured plan generation and introspection
4. **Skill Telemetry** - Per-skill execution metrics
5. **Error Recovery** - Automatic retry and fallback strategies

### Extension Points

Want to extend the system? Add to DIMOS, not here:

**Robot Skills**: Add to `MyUnitreeSkills` in DIMOS
**New Agent Type**: Implement DIMOS agent interface
**Custom Backend**: Add to DIMOS LLM client

## References

- **DIMOS Framework**: `src/dimos-unitree/`
- **Refactor Analysis**: `docs/AGENT_REFACTOR_ANALYSIS.md`
- **Vision Integration**: `docs/VISION_INTEGRATION_DESIGN.md`
- **Project Architecture**: `docs/project.md`
- **Tests**: `src/shadowhound_mission_agent/test/test_mission_executor.py`

## Summary

**Key Principles**:
1. **Separate ROS from business logic** - Enables testing and reuse
2. **Use DIMOS directly** - Don't wrap what's already abstracted
3. **Extend via skills** - Skills are the extension point
4. **Test without ROS** - Fast unit tests for business logic
5. **Follow conventions** - Use framework patterns, don't fight them

**Architecture Benefits**:
- ✅ Testable without ROS (10 unit tests in 0.09s)
- ✅ Reusable in non-ROS contexts (scripts, notebooks, web apps)
- ✅ Follows DIMOS conventions (skills as extension point)
- ✅ Clean separation of concerns (ROS vs business logic)
- ✅ Easy to understand and maintain (292 lines each component)
