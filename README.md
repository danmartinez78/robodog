# ShadowHound - Autonomous Mobile Robot with LLM Planning

An autonomous mobile robot system that combines ROS2 navigation with LLM/VLM-driven task planning for natural language mission execution on Unitree Go2.

**Status**: 🎯 Phase 0 (Bootstrap) - Package scaffolding in progress

---

## Quick Start

### 🚀 Fastest Start (Recommended)

```bash
# 1. Clone the repository
git clone https://github.com/danmartinez78/shadowhound.git
cd shadowhound

# 2. Open in VS Code dev container
code .
# (Click "Reopen in Container" when prompted)

# 3. Run the start script
./start.sh --dev
```

That's it! The script handles everything:
- ✓ Checks dependencies
- ✓ Creates configuration (.env)
- ✓ Builds the workspace
- ✓ Launches the system
- ✓ Opens web dashboard at http://localhost:8080

Try a mission: "patrol around the room" 🤖

### 📚 For More Control

See [`SCRIPTS.md`](SCRIPTS.md) for all available scripts and options:
- `./start.sh` - Smart interactive launcher
- `./scripts/quick-start-dev.sh` - One-command development start
- `./scripts/check-deps.sh` - Verify dependencies
- `./scripts/test-web-only.sh` - Test web interface alone

### Prerequisites
- **Docker** with dev containers support
- **VS Code** with [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
- **OpenAI API Key** - Get one from [OpenAI Platform](https://platform.openai.com/api-keys)

### Dev Container Features
The container provides everything you need:
- ✅ ROS2 Humble Desktop Full
- ✅ Navigation2 stack
- ✅ CycloneDDS middleware
- ✅ Python tools (black, isort, pylint, mypy, pytest)
- ✅ DIMOS framework with LLM integration
- ✅ Helpful aliases (`cb`, `source-ws`, `rosdep-install`)

### Manual Setup (if needed)

```bash
# Inside the container terminal:
cb              # Build with colcon (alias for colcon build --symlink-install)
source-ws       # Source the workspace (alias for source install/setup.bash)

# Verify setup
ros2 pkg list | grep shadowhound
```

---

## Architecture Overview

ShadowHound uses a **four-layer architecture**:

```
┌─ Application ─┐  Launch files, configs, deployment
┌─ Agent ───────┐  LLM/VLM orchestration, mission planning
┌─ Skills ──────┐  Execution engine, safety, telemetry
┌─ Robot ───────┐  ROS2 bridge to go2_ros2_sdk
└─ Hardware ────┘  Unitree Go2 quadruped
```

### Core Packages (to be created)

- **`shadowhound_interfaces/`** - Custom ROS2 messages/services/actions
- **`shadowhound_robot/`** - Hardware interface layer
- **`shadowhound_skills/`** - Skills registry and implementations
- **`shadowhound_agent/`** - Mission planner and LLM integration
- **`shadowhound_bringup/`** - Launch files and configurations

See [`docs/project.md`](docs/project.md) for detailed architecture.

---

## Development Workflow

### Useful Aliases (Pre-configured)

```bash
cb              # colcon build --symlink-install
cbt             # colcon test
cbr             # colcon build && source install/setup.bash
source-ws       # source install/setup.bash (if workspace is built)
rosdep-install  # rosdep install --from-paths src --ignore-src -r -y
```

### Creating a New Package

```bash
cd src/
ros2 pkg create --build-type ament_python \
    --dependencies rclpy std_msgs \
    shadowhound_<name>

# Build and test
cb --packages-select shadowhound_<name>
source-ws
ros2 run shadowhound_<name> <node_name>
```

### Testing

```bash
# Run ROS2 tests
cbt --packages-select shadowhound_<name>

# Run Python unit tests
pytest src/shadowhound_<name>/test/

# Run specific test
pytest src/shadowhound_skills/test/test_registry.py -v
```

---

## Skills API Concept

The **Skills API** is the primary interface for robot control. Skills are typed, safe wrappers over ROS2 operations with built-in validation, timeouts, and telemetry.

### Example: Calling a Skill

```python
from shadowhound_skills import SkillRegistry

# Execute a navigation skill
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

### Example: Implementing a Skill

```python
from shadowhound_skills import Skill, SkillResult, register_skill

@register_skill("report.say")
class SaySkill(Skill):
    """Text-to-speech skill."""
    
    def validate_params(self, text: str) -> tuple[bool, str]:
        if not text or len(text) > 500:
            return False, "Text must be 1-500 characters"
        return True, ""
    
    def execute(self, text: str) -> SkillResult:
        # Implementation here
        return SkillResult(success=True, data={"spoken": text})
```

---

## Environment Variables

```bash
# ROS2 Configuration (pre-set in devcontainer)
ROS_DOMAIN_ID=42                      # Isolated network
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp # DDS implementation
RCUTILS_LOGGING_BUFFERED_STREAM=1     # Logging optimization

# Robot Connection (set when needed)
GO2_IP=192.168.1.103                  # Go2 robot IP
GO2_MODE=webrtc                       # webrtc or ethernet

# Agent Configuration (set when needed)
AGENT_BACKEND=cloud                   # cloud or local
OPENAI_API_KEY=<your-key>             # For cloud LLM
```

---

## Current Status & Roadmap

### ✅ Phase 0: Bootstrap (CURRENT)
- [x] Devcontainer with ROS2 Humble
- [x] Workspace structure
- [ ] Create package scaffolding
- [ ] First build and test

### 🔄 Phase 1: Basic Skills (NEXT)
- [ ] Skills registry implementation
- [ ] Basic skills (say, stop, rotate, snapshot)
- [ ] Unit tests

### 🔜 Phase 2: Robot Integration
- [ ] Import go2_ros2_sdk
- [ ] Robot interface implementation
- [ ] Hardware testing

### 🔮 Phase 3: Agent Integration
- [ ] LLM client
- [ ] Mission planner
- [ ] Natural language missions

See [`docs/project.md`](docs/project.md) for complete roadmap and implementation phases.

---

## Key Documentation

- **[`docs/project.md`](docs/project.md)** - Complete architecture and implementation plan
- **[`.github/copilot-instructions.md`](.github/copilot-instructions.md)** - AI agent development guide
- **Package READMEs** - Each package will have detailed documentation

---

## Troubleshooting

### Container Issues

**Problem**: Container fails to build
```bash
# Check Docker is running
docker ps

# Rebuild container from scratch
# Ctrl+Shift+P → "Dev Containers: Rebuild Container Without Cache"
```

**Problem**: Setup script fails
```bash
# Check setup.sh logs in terminal
# Common fix: permissions
sudo chown -R ros:ros /workspaces/shadowhound
```

### ROS Issues

**Problem**: Packages not found after build
```bash
# Always source after building
source-ws

# Or use the combined alias
cbr
```

**Problem**: Dependencies missing
```bash
# Install ROS dependencies
rosdep-install

# Check what's missing
rosdep check --from-paths src --ignore-src
```

**Problem**: Can't find ROS topics
```bash
# Check ROS environment
printenv | grep ROS

# List topics
ros2 topic list

# Check network (if using real robot)
ros2 daemon stop && ros2 daemon start
```

---

## Contributing

### Development Principles
1. **Container-First**: Always develop inside the devcontainer
2. **Skills-First**: Implement robot control as skills, not ad-hoc ROS code
3. **Safety-First**: Validate inputs, add timeouts, clamp velocities
4. **Test-Driven**: Write tests alongside implementation
5. **Document**: Keep `docs/project.md` updated

### Code Style
- **Python**: black (line length 99), isort, flake8, mypy
- **ROS2**: Follow [ROS2 conventions](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html)
- **Commits**: Use conventional commits (feat:, fix:, docs:, etc.)

### Before Submitting
```bash
# Format code
black src/shadowhound_*/shadowhound_*/ --line-length 99
isort src/shadowhound_*/shadowhound_*/

# Lint
flake8 src/shadowhound_*/shadowhound_*/ --max-line-length 99

# Type check
mypy src/shadowhound_*/shadowhound_*/

# Test
cbt
pytest src/
```

---

## License

[Add your license here]

## Acknowledgments

- Built on [go2_ros2_sdk](https://github.com/unitreerobotics/go2_ros2_sdk)
- Uses [ROS2 Humble](https://docs.ros.org/en/humble/)
- Inspired by LLM-based robotics research

---

**Ready to start developing?** Check [`docs/project.md`](docs/project.md) for the complete architecture and next steps!