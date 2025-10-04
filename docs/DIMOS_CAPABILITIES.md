# DIMOS Framework Capabilities

**Date**: October 4, 2025  
**Summary**: This document catalogs the existing capabilities in the DIMOS framework that ShadowHound can leverage.

---

## 1. Skills System

### Architecture
- **Location**: `dimos/skills/`
- **Base Classes**:
  - `AbstractSkill`: Base skill interface
  - `AbstractRobotSkill`: Robot-specific skills
  - `SkillLibrary`: Registry and management of skills

### Unitree Go2 Skills
**Location**: `dimos/robot/unitree/unitree_skills.py`

DIMOS provides **40+ built-in skills** for the Unitree Go2, including:

#### Locomotion Skills
- `BalanceStand`, `StandUp`, `StandDown`, `RecoveryStand`
- `Sit`, `RiseSit`
- `SwitchGait`, `ContinuousGait`, `EconomicGait`
- `SpeedLevel`, `BodyHeight`, `FootRaiseHeight`

#### Dynamic Maneuvers
- `FrontFlip`, `FrontJump`, `FrontPounce`
- `LeftFlip`, `RightFlip`
- `Handstand`, `CrossStep`, `Bound`

#### Expressive Behaviors
- `Hello`, `FingerHeart`, `Stretch`
- `Dance1`, `Dance2`, `WiggleHips`
- `Content`, `Wallow`, `Scrape`

#### Navigation
- `TrajectoryFollow`, `LeadFollow`
- `Euler` (orientation control)

#### System
- `GetState`, `Damp`, `Trigger`
- `SwitchJoystick`

### Visual Navigation Skills
**Location**: `dimos/skills/visual_navigation_skills.py`

Provides vision-based navigation capabilities.

---

## 2. Agent System

### Architecture
**Location**: `dimos/agents/`

### Base Agent Classes
- **`Agent`**: Base class with memory and subscription management
- **`LLMAgent`**: Extends Agent with LLM capabilities
- **`OpenAIAgent`**: Concrete implementation using OpenAI API

### Specialized Agents
- **`PlanningAgent`**: Multi-step task planning
- **`ClaudeAgent`**: Anthropic Claude integration
- **Local LLM Agents**:
  - `AgentHuggingfaceLocal`: Local HuggingFace models
  - `AgentHuggingfaceRemote`: Remote HuggingFace inference
  - `AgentCTransformersGGUF`: GGUF quantized models

### Agent Features
- **Semantic Memory**: ChromaDB-based context storage
- **Tool/Function Calling**: Integration with skill system
- **Streaming Support**: Async processing of inputs
- **Token Management**: Automatic context window handling
- **Prompt Building**: Template-based prompt construction

---

## 3. ROS2 Interfaces

### go2_interfaces Package
**Location**: `dimos/robot/unitree/external/go2_ros2_sdk/go2_interfaces/`

#### Messages
- **Robot Command**: `Go2Cmd.msg`, `Go2Move.msg`, `Go2RpyCmd.msg`
- **Robot State**: `Go2State.msg`, `IMU.msg`

### unitree_go Package
**Location**: `dimos/robot/unitree/external/go2_ros2_sdk/unitree_go/`

#### Comprehensive Messages
- **Sensor Data**: `IMUState.msg`, `LidarState.msg`
- **Motor Control**: `MotorCmd.msg`, `MotorCmds.msg`, `MotorState.msg`, `MotorStates.msg`
- **System**: `BmsCmd.msg`, `BmsState.msg`, `Error.msg`
- **Video**: `Go2FrontVideoData.msg`, `AudioData.msg`
- **Low-Level**: `LowCmd.msg`, `LowState.msg`
- **Terrain**: `HeightMap.msg`
- **Config**: `InterfaceConfig.msg`

**Result**: ShadowHound does NOT need custom interfaces - use existing go2_interfaces and unitree_go messages.

---

## 4. Robot Control

### UnitreeGo2 Class
**Location**: `dimos/robot/unitree/`

Main robot control interface that provides:
- High-level motion commands
- Sensor data access
- Video streaming
- Safety monitoring
- Skill execution

### UnitreeROSControl
ROS2 bridge for the robot, handling:
- Topic publishing/subscribing
- Service calls
- Action servers
- Mock robot support for testing

---

## 5. Perception System

### Available Pipelines
**Location**: `dimos/perception/`, `dimos/models/`

#### Detection
- **2D Object Detection**: COCO detector, Detic
- **Segmentation**: Various segmentation models
- **Depth Estimation**: Depth model support

#### Models
- **Detic**: Open-vocabulary object detection
- **Qwen**: Vision-Language model
- **Deformable-DETR**: Advanced detection (requires CUDA)
- **TorchScript Mask R-CNN**: Instance segmentation

**Note**: Some perception models require CUDA and may need to be skipped in CPU-only environments.

---

## 6. Path Planning

### Global Planner
**Location**: `dimos/robot/global_planner/`

### Local Planner
**Location**: `dimos/robot/local_planner/`

Provides navigation and obstacle avoidance capabilities.

---

## 7. Simulation Support

### Supported Simulators
**Location**: `dimos/simulation/`

- **Genesis**: Physics simulation
- **Isaac**: NVIDIA Isaac Sim integration
- **Base**: Common simulation interfaces

---

## 8. Web Interface

### DIMOS Web Dashboard
**Location**: `dimos/web/`

Provides:
- Robot telemetry visualization
- Video streaming
- Control interface
- WebSocket communication

---

## ShadowHound Integration Strategy

### What to Reuse (✅)
1. **All ROS2 Interfaces**: Use go2_interfaces and unitree_go messages
2. **DIMOS Skills**: Leverage UnitreeSkills directly
3. **Agent Framework**: Use OpenAIAgent and PlanningAgent
4. **Robot Control**: Use UnitreeGo2 and UnitreeROSControl
5. **Perception Pipelines**: Integrate existing detection/segmentation
6. **Path Planning**: Use DIMOS planners

### What to Create (🔨)
1. **shadowhound_mission_agent**: Thin wrapper around DIMOS agents for mission-specific logic
2. **shadowhound_bringup**: Launch files combining DIMOS + ShadowHound
3. **Custom Skills**: Only if needed for mission-specific behaviors not in DIMOS
4. **Mission Configurations**: YAML configs for different mission profiles

### What to Avoid (❌)
1. ~~Custom ROS interfaces~~ (use DIMOS interfaces)
2. ~~Custom robot control layer~~ (use UnitreeGo2)
3. ~~Custom skills framework~~ (use SkillLibrary)
4. ~~Custom agent base classes~~ (use OpenAIAgent)

---

## Next Steps

1. **Update shadowhound_mission_agent** to import and use DIMOS agents
2. **Create shadowhound_bringup** with launch files
3. **Test integration** with mock robot
4. **Document mission workflows** using DIMOS capabilities
