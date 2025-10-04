# ShadowHound AI Coding Agent Instructions

## Project Context

ShadowHound is an autonomous mobile robot system combining ROS2 navigation with LLM/VLM-driven planning. You're working on a **Unitree Go2 quadruped** that executes natural language missions through a typed **Skills API**.

**Current Phase**: Bootstrap (Phase 0) - Creating package scaffolding
**Next Milestone**: Basic skills implementation without robot hardware

---

## Architecture Quick Reference

### Four-Layer Stack
```
Application  → Launch files, configs, deployment
Agent        → LLM/VLM orchestration, mission planning
Skills       → Execution engine, safety, telemetry
Robot        → ROS2 bridge to go2_ros2_sdk hardware
```

### Package Map
- `shadowhound_interfaces/` - Custom ROS2 messages/services/actions
- `shadowhound_robot/` - Hardware interface layer (go2_ros2_sdk bridge)
- `shadowhound_skills/` - Skills registry + implementations
- `shadowhound_agent/` - Mission planner + LLM integration
- `shadowhound_bringup/` - Launch files and configurations

### Key Principles
1. **Skills-First**: All robot control through Skills API, never direct topic publishing
2. **Safety-First**: Every skill has timeout, validation, and error handling
3. **Container-First**: All development in devcontainer
4. **Type-First**: Use type hints, validate inputs, return structured results

---

## Development Workflow

### Standard Commands (Available in Devcontainer)
```bash
cb              # colcon build --symlink-install
source-ws       # source install/setup.bash
rosdep-install  # install ROS dependencies
cbt             # colcon test
cbr             # build and source
```

### Creating a New Package
```bash
cd src/
ros2 pkg create --build-type ament_python \
    --dependencies rclpy std_msgs \
    shadowhound_<name>

# Then: Edit package.xml, setup.py, implement nodes
cb --packages-select shadowhound_<name>
source-ws
```

### Testing Pattern
```python
# In test/test_<module>.py
import pytest
from shadowhound_skills import SkillRegistry

def test_skill_registration():
    skill = SkillRegistry.get("nav.rotate")
    assert skill is not None

def test_skill_execution():
    result = SkillRegistry.execute("nav.rotate", yaw=1.57)
    assert result.success == True
```

Run with: `pytest src/<package>/test/`

---

## Skills API Patterns

### Skill Implementation Template
```python
# In shadowhound_skills/skills/<category>.py
from shadowhound_skills.skill_base import Skill, SkillResult
from shadowhound_skills.skill_registry import register_skill

@register_skill("category.action_name")
class ActionSkill(Skill):
    """Brief description of what this skill does."""
    
    # Define parameters with types
    timeout: float = 30.0
    max_retries: int = 3
    
    def validate_params(self, **kwargs) -> tuple[bool, str]:
        """Validate input parameters before execution."""
        # Check required params exist and are valid
        if "param" not in kwargs:
            return False, "Missing required parameter 'param'"
        return True, ""
    
    def execute(self, **kwargs) -> SkillResult:
        """Execute the skill with safety and telemetry."""
        # 1. Validate
        valid, error = self.validate_params(**kwargs)
        if not valid:
            return SkillResult(success=False, error=error)
        
        # 2. Execute with timeout
        try:
            # Do the work
            result_data = self._do_work(**kwargs)
            
            # 3. Return structured result
            return SkillResult(
                success=True,
                data=result_data,
                telemetry={"duration_ms": 100}
            )
        except Exception as e:
            return SkillResult(success=False, error=str(e))
    
    def _do_work(self, **kwargs):
        """Internal implementation - interacts with robot interface."""
        # Access robot via self.robot_interface
        pass
```

### Calling Skills
```python
# From agent or test code
from shadowhound_skills import SkillRegistry

result = SkillRegistry.execute(
    "nav.goto",
    x=1.0, y=2.0, yaw=0.0,
    timeout=20.0
)

if result.success:
    print(f"Navigation complete: {result.data}")
else:
    print(f"Navigation failed: {result.error}")
```

---

## Robot Interface Patterns

### Accessing Robot State
```python
# In shadowhound_robot/robot_interface.py
class RobotInterface:
    """Bridge to go2_ros2_sdk topics/services."""
    
    def get_pose(self) -> tuple[float, float, float]:
        """Get current robot pose (x, y, yaw)."""
        # Subscribe to /odom, return latest
        pass
    
    def publish_velocity(self, linear: float, angular: float):
        """Publish velocity command (with safety clamps)."""
        # Clamp values to safe ranges
        linear = np.clip(linear, -0.5, 0.5)
        angular = np.clip(angular, -1.0, 1.0)
        # Publish to /cmd_vel
        pass
    
    def get_camera_image(self) -> np.ndarray:
        """Get latest camera image."""
        # Subscribe to /camera/compressed, decompress
        pass
```

### Skills Use Robot Interface
```python
# Skills never publish directly - they use RobotInterface
@register_skill("nav.rotate")
class RotateSkill(Skill):
    def execute(self, yaw: float) -> SkillResult:
        robot = self.get_robot_interface()
        
        start_yaw = robot.get_pose()[2]
        target_yaw = start_yaw + yaw
        
        # Control loop with timeout
        while not self.is_at_target(robot.get_pose()[2], target_yaw):
            robot.publish_velocity(linear=0.0, angular=0.3)
            if self.timeout_exceeded():
                return SkillResult(success=False, error="Timeout")
        
        robot.publish_velocity(0.0, 0.0)  # Stop
        return SkillResult(success=True)
```

---

## Agent Integration Patterns

### LLM Client Setup
```python
# In shadowhound_agent/models/llm_client.py
from openai import OpenAI

class LLMClient:
    def __init__(self, model: str = "gpt-4-turbo"):
        self.client = OpenAI()
        self.model = model
    
    def plan_mission(self, instruction: str) -> list[dict]:
        """Convert natural language to skill plan."""
        prompt = f"""Given the instruction: "{instruction}"
        Generate a sequence of skills to execute.
        
        Available skills: nav.goto, nav.rotate, perception.snapshot, report.say
        
        Return JSON list of skill calls."""
        
        response = self.client.chat.completions.create(
            model=self.model,
            messages=[{"role": "user", "content": prompt}],
            response_format={"type": "json_object"}
        )
        
        return json.loads(response.choices[0].message.content)
```

### Plan Execution
```python
# In shadowhound_agent/plan_executor.py
class PlanExecutor:
    def execute_plan(self, plan: list[dict]) -> bool:
        """Execute sequence of skill calls."""
        for step in plan:
            skill_name = step["name"]
            params = step.get("args", {})
            
            result = SkillRegistry.execute(skill_name, **params)
            
            if not result.success:
                self.logger.error(f"Skill {skill_name} failed: {result.error}")
                return False
            
            # Publish telemetry
            self.publish_step_result(step, result)
        
        return True
```

---

## Common Patterns & Anti-Patterns

### ✅ DO: Skills API
```python
# Good - type-safe, validated, telemetered
result = SkillRegistry.execute("nav.goto", x=1.0, y=2.0, yaw=0.0)
```

### ❌ DON'T: Direct ROS Publishing
```python
# Bad - no validation, no safety, no telemetry
cmd_vel_pub.publish(Twist(linear=Vector3(x=1.0)))
```

### ✅ DO: Structured Results
```python
return SkillResult(
    success=True,
    data={"distance_traveled": 5.2, "time_elapsed": 10.3},
    telemetry={"cpu_usage": 45.2}
)
```

### ❌ DON'T: Bare Returns
```python
return True  # Lost information!
```

### ✅ DO: Safety Validation
```python
def validate_params(self, x: float, y: float) -> tuple[bool, str]:
    if abs(x) > 10.0 or abs(y) > 10.0:
        return False, "Position out of safe bounds"
    return True, ""
```

### ❌ DON'T: Assume Valid Input
```python
def execute(self, x, y):
    # No checks - could crash or damage robot
    robot.move_to(x, y)
```

---

## Configuration & Environment

### Key Environment Variables
```bash
ROS_DOMAIN_ID=42                 # Isolated ROS network
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
GO2_IP=192.168.1.103             # Robot IP address
AGENT_BACKEND=cloud              # or 'local'
OPENAI_API_KEY=sk-...            # For cloud LLM
```

### Launch File Pattern
```python
# In shadowhound_bringup/launch/skills.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='shadowhound_skills',
            executable='skill_server',
            name='skill_server',
            parameters=[{
                'timeout_default': 30.0,
                'max_retries': 3,
            }]
        ),
    ])
```

---

## Phase-Specific Guidance

### Phase 0 (CURRENT): Bootstrap
**Focus**: Create package structure without hardware

**Tasks**:
1. Create `shadowhound_interfaces` with custom msgs/srvs
2. Create `shadowhound_robot` skeleton (stubs for now)
3. Create `shadowhound_skills` with registry + 2 basic skills
4. Create `shadowhound_agent` skeleton
5. Create `shadowhound_bringup` with launch files

**Testing**: Verify builds with `cb`, run unit tests with `pytest`

### Phase 1 (NEXT): Basic Skills
**Focus**: Implement testable skills without robot

**Skills to Implement**:
- `report.say(text)` - Log or TTS
- `nav.stop()` - Set velocity to zero
- `nav.rotate(yaw)` - Simple rotation mock
- `perception.snapshot()` - Capture test image

**Testing**: CLI skill execution, unit tests with mocks

---

## Troubleshooting

### Build Issues
```bash
# Clean build
rm -rf build install log
cb

# Check package dependencies
rosdep check --from-paths src --ignore-src

# Install missing deps
rosdep-install
```

### Import Errors
```bash
# Make sure workspace is sourced
source-ws

# Check Python path
python3 -c "import sys; print('\n'.join(sys.path))"

# Verify package installation
ros2 pkg list | grep shadowhound
```

### ROS Communication Issues
```bash
# Check ROS environment
printenv | grep ROS

# List active topics
ros2 topic list

# Monitor topic
ros2 topic echo /shadowhound/status
```

---

## Code Quality Requirements

### Required for All Python Code
- **Type hints**: All function signatures
- **Docstrings**: Google style for public APIs
- **Error handling**: Try/except with specific exceptions
- **Logging**: Use ROS logging (self.get_logger())
- **Testing**: Unit tests for all skills

### Pre-commit Checks (Automated)
```bash
# Format code
black src/shadowhound_*/shadowhound_*/ --line-length 99
isort src/shadowhound_*/shadowhound_*/

# Lint
flake8 src/shadowhound_*/shadowhound_*/ --max-line-length 99
pylint src/shadowhound_*/shadowhound_*/

# Type check
mypy src/shadowhound_*/shadowhound_*/
```

---

## Quick Reference

### File Locations
- Architecture: `docs/project.md`
- User guide: `README.md`
- Interfaces: `src/shadowhound_interfaces/`
- Skills: `src/shadowhound_skills/shadowhound_skills/skills/`
- Tests: `src/<package>/test/`

### Getting Help
- ROS2 Docs: https://docs.ros.org/en/humble/
- Project Context: `docs/project.md` (read this first!)
- Package README: Each package has README.md with details

---

**Remember**: Always check `docs/project.md` for the latest architecture and phase status before starting new work.
