# ShadowHound Project Context
_Last updated: 2025-10-04_

This document defines the architecture and implementation plan for ShadowHound: an autonomous mobile robot system that combines classical ROS2 navigation with LLM/VLM-driven task planning. 

**Built on [DIMOS (Dimensional Framework)](https://github.com/dimensionalOS/dimos-unitree)**: ShadowHound leverages the DIMOS framework for robot control, perception, and agent orchestration, focusing on mission-specific implementations rather than rebuilding infrastructure.

The system uses Unitree Go2 hardware via **go2_ros2_sdk** (through DIMOS), exposes capabilities through a typed **Skills API**, and orchestrates missions via natural language instructions.

---

## 🎯 Project Goals

**Primary Mission**: Enable natural language task execution on mobile robots
- **Target Robot**: Unitree Go2 (quadruped)
- **Target Environment**: Indoor spaces (homes, offices, labs)
- **Key Capability**: Multi-step autonomous missions combining navigation, perception, and reporting

**Example Mission**: _"Find the blue ball in the kitchen"_
1. Navigate to kitchen (using saved map or exploration)
2. Search for blue ball using vision (VLM or detector)
3. Approach target and report findings via speech + photo

---

## 🏗️ Architecture

### Layer Overview
```
┌─────────────────────────────────────────────────────────────┐
│ APPLICATION LAYER                                           │
│  • Launch files (bringup, configs)                          │
│  • Deployment scripts (docker-compose, cloud/edge)          │
└─────────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────────┐
│ AGENT LAYER (shadowhound_agent)                             │
│  • LLM/VLM orchestration                                    │
│  • Natural language → skill plans                           │
│  • Dialog and state management                              │
│  • Telemetry aggregation                                    │
└─────────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────────┐
│ SKILLS LAYER (shadowhound_skills)                           │
│  • Skills registry and execution engine                     │
│  • Skill implementations (nav, perception, reporting)       │
│  • Safety guards (timeouts, clamps, validation)             │
│  • Telemetry publishing                                     │
└─────────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────────┐
│ ROBOT INTERFACE (shadowhound_robot)                         │
│  • ROS2 topic/service/action bridge                         │
│  • go2_ros2_sdk integration                                 │
│  • State monitoring and feedback                            │
│  • QoS and transport configuration                          │
└─────────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────────┐
│ HARDWARE LAYER (go2_ros2_sdk - external)                    │
│  • /cmd_vel, /odom, /imu, /joint_states                     │
│  • Camera topics (compressed images)                        │
│  • Robot state and diagnostics                              │
└─────────────────────────────────────────────────────────────┘
```

### Data Flows

**Command Flow** (Top-Down):
```
User Instruction (text)
  ↓
DIMOS Agent: Parse & Plan (LLM/VLM)
  ↓
DIMOS SkillLibrary: Execute(skill, **params)
  ↓
UnitreeGo2 Robot: robot.move(), robot.webrtc_req()
  ↓
UnitreeROSControl: ROS2 Pub/Sub/Action
  ↓
Hardware: Actuators & Sensors
```

**Feedback Flow** (Bottom-Up):
```
Hardware: Sensor Data (ROS topics)
  ↓
UnitreeROSControl: State Aggregation (tf2, topics)
  ↓
UnitreeGo2: Reactive Streams (RxPY)
  ↓
DIMOS Skills: Result + Telemetry
  ↓
DIMOS Agent: Status Updates & Replanning
  ↓
User: Speech/Visual Reports (FastAPI)
```

---

## 🔗 DIMOS Integration

### What DIMOS Provides Out-of-the-Box

**Robot Control** (`dimos.robot.unitree.UnitreeGo2`):
```python
from dimos.robot.unitree import UnitreeGo2, UnitreeROSControl

robot = UnitreeGo2(
    ip=os.getenv('ROBOT_IP'),
    ros_control=UnitreeROSControl(),
)

# Direct control methods
robot.move(distance=1.0, speed=0.5)
robot.rotate(angle=1.57)
robot.webrtc_req(api_id=1006)  # RecoveryStand
robot.get_pose()  # Returns (position, rotation)
```

**Skills Framework** (`dimos.skills`):
```python
from dimos.skills.navigation import NavigateToGoal, NavigateWithText
from dimos.skills import SkillLibrary

# Skills are already implemented!
robot_skills = robot.get_skills()
robot_skills.add(NavigateToGoal)
robot_skills.create_instance("NavigateToGoal", robot=robot)

# Execute via library
result = robot_skills.execute("NavigateToGoal", x=1.0, y=2.0)
```

**Agent Integration** (`dimos.agents`):
```python
from dimos.agents import OpenAIAgent

agent = OpenAIAgent(
    dev_name="ShadowHoundAgent",
    tools=robot_skills.get_tools(),  # Auto-generates tool definitions
    model="gpt-4-turbo"
)

# Agent can now call robot skills via function calling
response = agent.query("Navigate to the kitchen")
```

**Perception Pipelines**:
- Object detection (`ObjectDetectionStream` with Detic)
- Person tracking (`PersonTrackingStream`)
- Spatial memory (`SpatialMemory` with ChromaDB)
- Video streams (ROS topics → RxPY observables)

**Path Planning**:
- Local: `VFHPurePursuitPlanner` (obstacle avoidance)
- Global: `AstarPlanner` (costmap-based)

### What ShadowHound Adds

**Custom Skills** (mission-specific):
- `PatrolArea(waypoints)` - Patrol predefined area
- `SearchForObject(object_name, search_area)` - Systematic search
- `InvestigateAnomaly(location)` - Respond to detections
- `ReturnToBase()` - Navigate home and dock
- `ReportFindings(format)` - Generate mission reports

**Mission Planning**:
- Higher-level task decomposition
- Multi-mission scheduling
- Context-aware behavior selection

**Deployment Configuration**:
- ROS2 workspace integration
- Launch file orchestration
- Hardware-specific tuning

---

## 📦 Package Structure

**Simplified structure leveraging DIMOS**:

### shadowhound_bringup
**Type**: ament_cmake  
**Purpose**: Launch files and mission configurations

```
shadowhound_bringup/
├── launch/
│   ├── full.launch.py         # Complete system with DIMOS
│   ├── patrol.launch.py       # Patrol mission
│   └── search.launch.py       # Search mission
├── config/
│   ├── robot_config.yaml      # Robot parameters
│   ├── mission_params.yaml    # Mission-specific settings
│   └── skills_config.yaml     # Custom skill configurations
└── CMakeLists.txt
```

### shadowhound_skills
**Type**: ament_python  
**Purpose**: Mission-specific skills extending DIMOS

```
shadowhound_skills/
└── shadowhound_skills/
    ├── __init__.py
    ├── patrol.py              # PatrolArea skill
    ├── search.py              # SearchForObject skill
    ├── investigation.py       # InvestigateAnomaly skill
    └── reporting.py           # ReportFindings skill
```

### shadowhound_mission_agent (existing)
**Type**: ament_python  
**Purpose**: High-level mission orchestration

```
shadowhound_mission_agent/
└── shadowhound_mission_agent/
    ├── mission_agent.py       # Main mission controller
    ├── mission_types.py       # Mission definitions
    └── planner.py             # Mission planning logic
```

**Note**: We NO LONGER need separate packages for:
- ❌ `shadowhound_interfaces` - Use DIMOS message types
- ❌ `shadowhound_robot` - Use `UnitreeGo2` from DIMOS
- ❌ Core navigation/perception skills - Use DIMOS implementations

---

## �� Skills API Reference

### Core Concepts

**Skill**: A typed, safe function that wraps ROS2 operations
- **Input**: Validated parameters (type hints)
- **Output**: Result dict + telemetry
- **Guarantees**: Timeout, safety checks, error handling

**Skill Signature**:
```python
@register_skill("category.action")
class MySkill(Skill):
    def validate_params(self, **kwargs) -> tuple[bool, str]:
        """Check params before execution."""
        pass
    
    def execute(self, **kwargs) -> SkillResult:
        """Execute the skill with safety."""
        pass
```

**SkillResult**:
```python
@dataclass
class SkillResult:
    success: bool
    data: dict = field(default_factory=dict)
    error: str = ""
    telemetry: dict = field(default_factory=dict)
```

### MVP Skills

#### Navigation & Mapping
- `nav.goto(x, y, yaw)` - Move to waypoint
- `nav.rotate(yaw)` - Rotate in place
- `nav.stop()` - Emergency stop
- `map.save(name)` / `map.load(name)` - Map management

#### Perception
- `perception.snapshot()` - Capture image
- `perception.detect(object)` - Object detection
- `perception.describe_scene()` - Scene understanding

#### Reporting & UX
- `report.say(text)` - Text-to-speech
- `report.caption(image)` - Image captioning
- `report.alert(level, message)` - System alerts

### Example Usage

```python
from shadowhound_skills import SkillRegistry

# Execute a skill
result = SkillRegistry.execute("nav.goto", x=1.0, y=2.0, yaw=0.0)

if result.success:
    print(f"Navigation complete: {result.data}")
else:
    print(f"Navigation failed: {result.error}")
```

---

## 🗺️ Implementation Phases

### Phase 0: DIMOS Integration (CURRENT) 🎯
**Goal**: Integrate DIMOS framework into ShadowHound workspace

**Tasks**:
- [x] Devcontainer with ROS2 Humble
- [x] Workspace structure and build system
- [ ] Add dimos-unitree to dependencies (`shadowhound.repos`)
- [ ] Add go2_ros2_sdk as external dependency
- [ ] Create `shadowhound_bringup` with DIMOS launch files
- [ ] Verify DIMOS imports work in workspace
- [ ] First successful build with DIMOS

**Exit Criteria**: Can initialize `UnitreeGo2` robot class and run DIMOS examples

---

### Phase 1: Custom Skills 🔄
**Goal**: Implement ShadowHound-specific skills

**Tasks**:
- [ ] Create `shadowhound_skills` package
- [ ] Implement `PatrolArea` skill
- [ ] Implement `SearchForObject` skill
- [ ] Implement `InvestigateAnomaly` skill
- [ ] Implement `ReportFindings` skill
- [ ] Unit tests for custom skills
- [ ] Integration with DIMOS SkillLibrary

**Exit Criteria**: Custom skills can be called via DIMOS agent

---

### Phase 2: Hardware Testing 🔄
**Goal**: Test DIMOS integration with real Go2 robot

**Tasks**:
- [ ] Test basic DIMOS robot control (move, rotate, webrtc)
- [ ] Test DIMOS navigation skills (NavigateToGoal)
- [ ] Test perception streams (camera, object detection)
- [ ] Test custom ShadowHound skills on hardware
- [ ] Safety validation and emergency stop testing
- [ ] Performance tuning

**Exit Criteria**: Can execute multi-step missions on real robot

---

### Phase 3: Mission Agent 🔄
**Goal**: High-level mission orchestration

**Tasks**:
- [ ] Extend `shadowhound_mission_agent` to use DIMOS agents
- [ ] Mission type definitions (patrol, search, investigation)
- [ ] Mission planning logic
- [ ] Multi-mission scheduling
- [ ] Status reporting and telemetry
- [ ] Web interface integration (DIMOS FastAPI)

**Exit Criteria**: Can execute complex missions from natural language

---

### Phase 4: Advanced Perception 🔜
**Focus**: Leverage DIMOS perception for mission tasks

**Tasks**:
- [ ] Configure object detection for mission-specific classes
- [ ] Integrate spatial memory for location recall
- [ ] VLM-based scene understanding
- [ ] Visual navigation using DIMOS `NavigateWithText`
- [ ] Person tracking for security missions

---

### Phase 5: Deployment Optimization 🔜
**Focus**: Production deployment and performance

**Tasks**:
- [ ] Local LLM deployment (optional, for offline operation)
- [ ] Docker compose for full system
- [ ] Multi-robot coordination (optional)
- [ ] Battery-aware mission planning
- [ ] Logging and diagnostics
- [ ] OTA updates

---

### Phase 6: Advanced Missions 🔮
**Focus**: Complex multi-stage missions

**Tasks**:
- [ ] Multi-room search patterns
- [ ] Dynamic replanning on failures
- [ ] Human interaction skills
- [ ] Environmental monitoring missions
- [ ] Long-duration autonomous operation

---

## 🚀 Development Workflow

### Quick Commands (Devcontainer)
```bash
cb              # colcon build --symlink-install
source-ws       # source install/setup.bash
rosdep-install  # rosdep install --from-paths src --ignore-src -y
cbt             # colcon test
cbr             # build and source
```

### Working with DIMOS

**Install DIMOS** (via shadowhound.repos):
```bash
cd /workspaces/shadowhound
vcs import src < shadowhound.repos
pip3 install -e src/dimos-unitree
```

**Test DIMOS Integration**:
```python
# In Python REPL or script
from dimos.robot.unitree import UnitreeGo2, UnitreeROSControl

# This should work if integration is successful
robot = UnitreeGo2(
    ros_control=UnitreeROSControl(mock_connection=True),
    disable_video_stream=True
)
print("DIMOS integration successful!")
```

### Creating Custom Skills
```bash
cd src/shadowhound_skills/shadowhound_skills/
# Create new skill file
cat > patrol.py << 'EOF'
from dimos.skills.skills import AbstractRobotSkill

class PatrolArea(AbstractRobotSkill):
    name = "PatrolArea"
    description = "Patrol predefined waypoints"
    # ... implementation
EOF

# Build and test
cb --packages-select shadowhound_skills
pytest src/shadowhound_skills/test/
```

### Testing
```bash
# Unit tests (Python)
pytest src/shadowhound_skills/test/

# ROS tests
cbt --packages-select shadowhound_skills

# Integration tests with DIMOS
python3 src/shadowhound_skills/examples/test_patrol.py
```

---

## ⚙️ Configuration

### Key Environment Variables
```bash
# ROS Configuration
ROS_DOMAIN_ID=42                      # Isolated ROS network
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp # DDS implementation

# Robot Connection
ROBOT_IP=192.168.1.103                # Robot IP address
CONN_TYPE=webrtc                      # Connection type (webrtc/local)

# Agent Configuration
OPENAI_API_KEY=<key>                  # For OpenAI models
ANTHROPIC_API_KEY=<key>               # For Claude models (optional)
AGENT_BACKEND=cloud                   # 'cloud' or 'local'

# DIMOS Configuration
ROS_OUTPUT_DIR=./assets/output/ros    # Output directory for logs/data
```

### Launch Modes

**Development with Laptop (WebRTC)**:
```bash
# Set environment
export ROBOT_IP=192.168.1.103
export CONN_TYPE=webrtc
export OPENAI_API_KEY=sk-...

# Launch ShadowHound with DIMOS
ros2 launch shadowhound_bringup full.launch.py
```

**Testing Without Robot**:
```bash
# Mock robot connection
ros2 launch shadowhound_bringup skills_test.launch.py mock:=true
```

**Run DIMOS Examples**:
```bash
# Navigate to DIMOS tests
cd src/dimos-unitree/tests

# Run basic robot test
python3 test_robot.py

# Run agent with web interface
python3 test_unitree_agent_queries_fastapi.py
```

---

## 🧪 Safety & Constraints

### DIMOS Safety Mechanisms (Built-in)
1. **Velocity Clamping**: DIMOS `UnitreeROSControl` clamps velocities to safe ranges
2. **Command Queue**: `ROSCommandQueue` manages multi-step actions safely
3. **Emergency Stop**: `KillSkill()` for immediate halt
4. **WebRTC Safety**: Recovery stand, fall protection via webrtc APIs
5. **Transform Management**: tf2-based localization for safe navigation

### ShadowHound Safety Additions
- **Mission Timeouts**: Max execution time per mission
- **Geofencing**: Patrol/search area boundaries
- **Battery Monitoring**: Auto-return to base at threshold
- **Human Detection**: Slow down when person nearby
- **Error Recovery**: Automatic retry with backoff

### Performance Constraints (DIMOS Defaults)
- **Control Loop**: 50 Hz (via ROS2)
- **Camera**: 10-30 Hz (configurable)
- **IMU**: 100 Hz
- **Odometry**: 50 Hz
- **Max Linear Vel**: 0.5 m/s (DIMOS default, tunable)
- **Max Angular Vel**: 1.0 rad/s (DIMOS default, tunable)

### Key ROS2 Topics (via DIMOS)
```
# Control (DIMOS UnitreeROSControl)
/cmd_vel_out              # Motion commands
/pose_cmd                 # Pose commands
/webrtc_req               # WebRTC commands
/rt/api/sport/request     # Sport mode API

# Feedback (go2_ros2_sdk)
/odom                     # Robot odometry
/imu                      # IMU data
/go2_states               # Robot state
/camera/compressed        # Camera feed
/camera/camera_info       # Camera calibration

# Navigation (DIMOS planners)
/local_costmap/costmap    # Local costmap
/map                      # Global map
/tf                       # Transform tree
/tf_static                # Static transforms

# DIMOS Internals (RxPY Observables)
# These are Python streams, not ROS topics:
# - video_stream_ros
# - object_detection_stream
# - person_tracking_stream
# - local_planner_viz_stream
```

---

## 📚 References

### Project Documentation
- `docs/project.md` (this file) - Architecture and DIMOS integration
- `.github/copilot-instructions.md` - AI agent development guide
- `README.md` - Quick start and user guide
- `shadowhound.repos` - VCS dependencies (DIMOS, go2_ros2_sdk)

### DIMOS Framework
- [DIMOS GitHub](https://github.com/dimensionalOS/dimos-unitree) - Main repository
- [DIMOS README](https://github.com/dimensionalOS/dimos-unitree/blob/main/README.md) - Framework overview
- [DIMOS Examples](https://github.com/dimensionalOS/dimos-unitree/tree/main/tests) - Test files showing usage
- `dimos.robot.unitree.UnitreeGo2` - Main robot class
- `dimos.skills` - Skills framework
- `dimos.agents` - Agent implementations

### Underlying Technologies
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [go2_ros2_sdk](https://github.com/dimensionalOS/go2_ros2_sdk) - DIMOS fork
- [Unitree Go2 Docs](https://support.unitree.com/home/en/Go2_developer)
- [OpenAI API](https://platform.openai.com/docs)
- [RxPY (ReactiveX)](https://rxpy.readthedocs.io/) - Used by DIMOS for streams

---

## 🤝 Contributing

### Design Principles
1. **DIMOS-First**: Use DIMOS infrastructure, don't rebuild
2. **Mission-Specific**: Focus on ShadowHound's unique requirements
3. **Skills-Based**: Extend AbstractRobotSkill for new capabilities
4. **Safety-First**: Validate inputs, handle errors, respect limits
5. **Test-Driven**: Write tests for custom skills
6. **Document**: Keep project.md updated with DIMOS integration details

### Code Style
- **Python**: black (99 chars), isort, flake8
- **ROS2**: Follow [ROS2 style guide](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html)
- **Commits**: [Conventional commits](https://www.conventionalcommits.org/)
- **Docstrings**: Google style (match DIMOS conventions)

### Development Workflow
1. Check DIMOS for existing functionality first
2. Implement ShadowHound-specific features as extensions
3. Create custom skills inheriting from `AbstractRobotSkill`
4. Test with DIMOS mock mode before hardware
5. Update documentation

### Pull Request Process
1. Create feature branch: `git checkout -b feature/my-mission-skill`
2. Implement changes with tests
3. Run linters and tests: `cb && pytest`
4. Test with DIMOS mock mode
5. Update documentation (this file)
6. Submit PR with clear description

---

## 🔄 Version History

- **v0.2.0** (2025-10-04): DIMOS integration redesign
  - Adopted DIMOS framework for robot control, skills, and agents
  - Simplified architecture to focus on mission-specific implementations
  - Updated packages: removed interfaces/robot, kept bringup/skills/mission_agent
  - Phase 0 now focuses on DIMOS integration instead of scaffolding
  
- **v0.1.0** (2025-10-04): Initial architecture design
  - Defined 4-layer architecture
  - Specified 5 core packages (now simplified to 3)
  - Outlined 6 implementation phases

---

_This is the source of truth for ShadowHound architecture. Built on [DIMOS](https://github.com/dimensionalOS/dimos-unitree)._
