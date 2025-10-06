# Agent Architecture

## Overview

The ShadowHound agent module provides a clean, decoupled architecture for AI agents that control the robot. It separates agent logic from ROS orchestration, making the code more testable, maintainable, and extensible.

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                      Mission Agent Node (ROS)                   │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │                      AgentFactory                         │  │
│  │  Creates agents based on configuration                    │  │
│  └───────────────────────────────────────────────────────────┘  │
│                              │                                   │
│                              ▼                                   │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │                      BaseAgent                            │  │
│  │  Abstract interface for all agents                        │  │
│  │  - execute_mission(command) -> MissionResult              │  │
│  │  - plan_mission(command) -> List[SkillCall]               │  │
│  └───────────────────────────────────────────────────────────┘  │
│                              │                                   │
│              ┌───────────────┼───────────────┐                  │
│              ▼               ▼               ▼                  │
│      ┌──────────────┐ ┌──────────────┐ ┌──────────────┐        │
│      │ OpenAIAgent  │ │PlanningAgent │ │ CustomAgent  │        │
│      │ (GPT-4)      │ │(DIMOS Plan)  │ │ (Your impl)  │        │
│      └──────────────┘ └──────────────┘ └──────────────┘        │
│              │               │               │                  │
│              └───────────────┼───────────────┘                  │
│                              ▼                                   │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │                   Robot Skills API                        │  │
│  │  - nav.goto, nav.rotate, perception.snapshot, etc.        │  │
│  └───────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

## Core Components

### 1. BaseAgent (Abstract)

The foundation interface that all agents must implement.

```python
from shadowhound_mission_agent.agent import BaseAgent, MissionResult, SkillCall

class MyCustomAgent(BaseAgent):
    def execute_mission(self, command: str) -> MissionResult:
        """Execute a natural language command."""
        # Your implementation
        return MissionResult(
            status=MissionStatus.SUCCESS,
            message="Mission completed",
            skills_executed=[...],
            telemetry={...}
        )
    
    def plan_mission(self, command: str) -> list[SkillCall]:
        """Generate a plan without executing."""
        # Your implementation
        return [SkillCall(...), SkillCall(...)]
```

**Key Methods:**
- `execute_mission(command: str) -> MissionResult` - Execute a command and return structured result
- `plan_mission(command: str) -> list[SkillCall]` - Generate execution plan (optional, returns [] by default)

**Attributes:**
- `skills` - SkillLibrary instance for robot control
- `robot` - Robot interface instance
- `config` - Configuration dict (model, dev_name, etc.)

### 2. MissionResult (Dataclass)

Structured result from mission execution with telemetry and status tracking.

```python
@dataclass
class MissionResult:
    status: MissionStatus  # SUCCESS, FAILURE, PARTIAL, CANCELLED
    message: str  # Human-readable outcome description
    skills_executed: List[SkillCall] = None  # Successfully executed skills
    skills_failed: List[SkillCall] = None  # Failed skills
    raw_response: Optional[str] = None  # Raw AI response (debugging)
    telemetry: Dict[str, Any] = None  # Metrics (time, tokens, etc.)
```

**Example:**
```python
result = agent.execute_mission("Go to the kitchen")

if result.status == MissionStatus.SUCCESS:
    print(f"✅ {result.message}")
    print(f"Executed {len(result.skills_executed)} skills")
    print(f"Telemetry: {result.telemetry}")
else:
    print(f"❌ {result.message}")
    print(f"Failed skills: {result.skills_failed}")
```

### 3. AgentFactory

Factory pattern for creating agent instances based on type string.

```python
from shadowhound_mission_agent.agent import AgentFactory

# Create agent
agent = AgentFactory.create(
    agent_type="openai",  # or "planning", "custom"
    skills=my_skills,
    robot=my_robot,
    config={
        "dev_name": "shadowhound",
        "model": "gpt-4-turbo",
    }
)

# Or from config file
agent = AgentFactory.from_config_dict({
    "agent_type": "openai",
    "dev_name": "shadowhound",
    "model": "gpt-4-turbo"
})
```

**Registering Custom Agents:**
```python
class MyLocalLLMAgent(BaseAgent):
    # Your implementation
    pass

# Register with factory
AgentFactory.register_agent_type("local_llm", MyLocalLLMAgent)

# Now you can create it
agent = AgentFactory.create(agent_type="local_llm", ...)
```

### 4. Built-in Agent Implementations

#### OpenAIAgent
Wraps DIMOS OpenAIAgent for GPT-4/GPT-3.5 mission execution.

```python
agent = AgentFactory.create(
    agent_type="openai",
    skills=skills,
    robot=robot,
    config={"model": "gpt-4-turbo"}
)
```

**Features:**
- Uses OpenAI API (requires `OPENAI_API_KEY`)
- Handles Observable pattern from DIMOS
- Returns structured MissionResult
- Automatic error handling

#### PlanningAgent
Wraps DIMOS PlanningAgent for explicit planning before execution.

```python
agent = AgentFactory.create(
    agent_type="planning",
    skills=skills,
    robot=robot,
    config={"dev_name": "shadowhound"}
)

# Plan first (optional)
plan = agent.plan_mission("Patrol the perimeter")
print(f"Plan: {plan}")

# Execute
result = agent.execute_mission("Patrol the perimeter")
```

**Features:**
- Separate planning and execution phases
- Better transparency into AI decisions
- Plan validation before execution

## Usage in Mission Agent Node

The `MissionAgentNode` uses the factory to create agents based on ROS parameters:

```python
# In MissionAgentNode.__init__()

# Read ROS parameters
agent_type = "planning" if self.use_planning else "openai"

# Create agent via factory
self.agent = AgentFactory.create(
    agent_type=agent_type,
    skills=self.skills,
    robot=self.robot,
    config={
        "dev_name": "shadowhound",
        "agent_type": "Mission",
        "model": "gpt-4-turbo"
    }
)

# Execute missions through unified interface
def mission_callback(self, msg):
    result: MissionResult = self.agent.execute_mission(msg.data)
    
    if result.status == MissionStatus.SUCCESS:
        self.get_logger().info(f"Mission success: {result.message}")
    else:
        self.get_logger().error(f"Mission failed: {result.message}")
```

## Benefits of This Architecture

### 1. **Testability**
- Test agents without ROS, robot hardware, or DIMOS
- Mock agents for integration testing
- Isolated unit tests for each component

```python
# Example test
def test_agent_execution():
    agent = MockAgent()
    result = agent.execute_mission("test command")
    
    assert result.status == MissionStatus.SUCCESS
    assert len(result.skills_executed) == 1
```

### 2. **Extensibility**
- Easy to add new agent types (local LLMs, custom planners, etc.)
- Register custom agents without modifying core code
- Swap agents via configuration

```python
# Add new agent type
AgentFactory.register_agent_type("llamacpp", LlamaCppAgent)
```

### 3. **Observability**
- Structured results with status, telemetry, and skill tracking
- Better logging and debugging
- Metrics for monitoring (execution time, token usage, etc.)

### 4. **Separation of Concerns**
- Agent logic decoupled from ROS orchestration
- Mission node becomes thin layer for ROS integration
- Agent implementations focus on AI/planning logic

### 5. **Consistency**
- All agents share same interface
- Uniform error handling
- Predictable behavior across agent types

## Adding a New Agent Type

Follow these steps to add a new agent implementation:

### 1. Create Agent Class

```python
# In shadowhound_mission_agent/agent/my_custom_agent.py

from .base_agent import BaseAgent, MissionResult, MissionStatus, SkillCall

class MyCustomAgent(BaseAgent):
    """My custom agent implementation."""
    
    def __init__(self, skills, robot, config):
        super().__init__(skills, robot, config)
        # Your initialization
        self.model = config.get("model", "default")
    
    def execute_mission(self, command: str) -> MissionResult:
        """Execute mission with custom logic."""
        try:
            # Your execution logic
            skills_used = [...]
            
            return MissionResult(
                status=MissionStatus.SUCCESS,
                message=f"Completed: {command}",
                skills_executed=skills_used,
                telemetry={"execution_time": 1.2}
            )
        except Exception as e:
            return MissionResult(
                status=MissionStatus.FAILURE,
                message=f"Error: {str(e)}"
            )
    
    def plan_mission(self, command: str) -> list[SkillCall]:
        """Generate plan (optional)."""
        # Your planning logic
        return []
```

### 2. Export from Module

```python
# In shadowhound_mission_agent/agent/__init__.py

from .my_custom_agent import MyCustomAgent

__all__ = [
    # ... existing exports
    "MyCustomAgent",
]
```

### 3. Register with Factory

```python
# In shadowhound_mission_agent/agent/agent_factory.py

from .my_custom_agent import MyCustomAgent

_AGENT_TYPES = {
    "openai": OpenAIAgent,
    "planning": PlanningAgent,
    "custom": MyCustomAgent,  # Add here
}
```

### 4. Use in Configuration

```python
# Create agent
agent = AgentFactory.create(
    agent_type="custom",
    skills=skills,
    robot=robot,
    config={"model": "my-model"}
)

# Or via config file
config = {
    "agent_type": "custom",
    "model": "my-model"
}
agent = AgentFactory.from_config_dict(config)
```

## Configuration Examples

### Using OpenAI Agent (Cloud)

```python
agent = AgentFactory.create(
    agent_type="openai",
    skills=skills,
    robot=robot,
    config={
        "dev_name": "shadowhound",
        "model": "gpt-4-turbo",  # or "gpt-3.5-turbo"
    }
)
```

Environment: `OPENAI_API_KEY=sk-...`

### Using Planning Agent

```python
agent = AgentFactory.create(
    agent_type="planning",
    skills=skills,
    robot=robot,
    config={
        "dev_name": "shadowhound",
        "agent_type": "Planning",
    }
)
```

### Using Custom Local LLM

```python
# After registering custom agent
agent = AgentFactory.create(
    agent_type="llamacpp",
    skills=skills,
    robot=robot,
    config={
        "model_path": "/models/llama-7b.gguf",
        "context_length": 2048,
    }
)
```

## Error Handling

All agents should return structured results, never raise exceptions to the caller:

```python
def execute_mission(self, command: str) -> MissionResult:
    try:
        # Execution logic
        return MissionResult(status=MissionStatus.SUCCESS, ...)
    except SpecificError as e:
        # Handle specific errors
        return MissionResult(
            status=MissionStatus.FAILURE,
            message=f"Specific error: {e}"
        )
    except Exception as e:
        # Catch-all for unexpected errors
        return MissionResult(
            status=MissionStatus.FAILURE,
            message=f"Unexpected error: {e}",
            telemetry={"error_type": type(e).__name__}
        )
```

## Testing

### Unit Testing Agents

```python
# test/test_agent_module.py

from shadowhound_mission_agent.agent import (
    BaseAgent, MissionResult, MissionStatus, AgentFactory
)

def test_custom_agent():
    """Test custom agent without ROS/robot."""
    agent = AgentFactory.create(
        agent_type="custom",
        skills=MockSkills(),
        robot=MockRobot(),
        config={}
    )
    
    result = agent.execute_mission("test command")
    
    assert result.status == MissionStatus.SUCCESS
    assert result.message is not None
    assert result.telemetry is not None
```

### Integration Testing with ROS

```python
# Launch test node with mock agent
node = MissionAgentNode()
node.agent = MockAgent()  # Inject mock

# Send mission command
msg = String(data="Go forward")
node.mission_callback(msg)

# Verify result published to /mission_status
```

## Future Enhancements

- **Vision-enabled agents**: Integrate GPT-4 Vision for image-based missions
- **Local LLM support**: Add Llama, Mistral, etc. for offline operation
- **Multi-agent coordination**: Agents that collaborate on complex missions
- **Learning agents**: RL-based agents that improve over time
- **Skill recommendation**: Agents suggest new skills based on mission patterns

## References

- **Base Implementation**: `shadowhound_mission_agent/agent/base_agent.py`
- **Factory Pattern**: `shadowhound_mission_agent/agent/agent_factory.py`
- **OpenAI Wrapper**: `shadowhound_mission_agent/agent/openai_agent.py`
- **Planning Wrapper**: `shadowhound_mission_agent/agent/planning_agent.py`
- **Usage Example**: `shadowhound_mission_agent/mission_agent.py`
- **Tests**: `test/test_agent_basic.py`

---

**Last Updated**: 2025-01-05
**Maintainer**: ShadowHound Development Team
