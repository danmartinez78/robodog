# ShadowHound 🐕‍🦺

**An ambitious robotic platform built on the Unitree GO2 quadruped with advanced AI integration**

[![Python](https://img.shields.io/badge/python-3.10-blue.svg)](https://docs.python.org/3/whatsnew/3.10.html)
[![ROS2](https://img.shields.io/badge/ROS2-Iron%20%7C%20Humble-green.svg)](https://docs.ros.org/en/iron/)
[![License](https://img.shields.io/badge/license-BSD--2-yellow.svg)](LICENSE)

---

## 🎯 Project Vision

ShadowHound aims to create a general-purpose, interactive robotic platform that can:
- Navigate autonomously in indoor environments
- Interact naturally with humans via voice and gestures  
- Use AI models for perception, reasoning, and action
- Serve as a research platform for embodied AI

## 🏗️ System Architecture

### Hardware Stack
- **GO2 Pro (jailbroken)**: Mobile quadruped base with sensors/actuators
- **Jetson Orin Nano 8GB**: Mounted on GO2 for local autonomy (SLAM, perception, control)
- **Jetson AGX Thor/GPU PC**: High-power compute for LLM/VLM/VLA models

### Software Stack
- **Base**: Fork of [`abizovnuralem/go2_ros2_sdk`](https://github.com/abizovnuralem/go2_ros2_sdk)
- **Framework**: ROS2 (Iron/Humble) with CyclonDX
- **AI Integration**: gRPC interface to LLM/VLM models
- **Networking**: Distributed reasoning over Wi-Fi/Ethernet

## 📦 Project Structure

```
├── go2_robot_sdk/           # Core robot interface and drivers
├── go2_interfaces/          # Custom ROS2 message definitions  
├── coco_detector/           # Real-time object detection
├── lidar_processor/         # LiDAR processing and point clouds
├── speech_processor/        # Speech recognition and synthesis
├── docs/                    # Documentation and project context
└── docker/                  # Containerization support
```

## 🚀 Current Capabilities

✅ **Robot Control**: Full URDF, joint states, IMU sync, joystick control  
✅ **Perception**: LiDAR point cloud, camera streams, laser scan  
✅ **Navigation**: SLAM (slam_toolbox) and Navigation (nav2)  
✅ **Detection**: Object detection with COCO dataset  
✅ **Connectivity**: Multi-robot support, WebRTC/CycloneDDS protocols  

## 🛣️ Development Roadmap

### Phase 0: System Bring-up (Current)
- [x] Hardware integration and ROS2 setup
- [x] Basic autonomy (SLAM, navigation, object detection)
- [ ] Joystick control debugging and optimization
- [ ] System stability and performance tuning

### Phase 1: Interactive Capabilities
- [ ] Voice command processing and response
- [ ] Gesture recognition and interpretation
- [ ] Basic human-robot interaction patterns

### Phase 2: Reasoning Integration  
- [ ] Distributed reasoning architecture (Orin ↔ Thor/PC)
- [ ] LLM/VLM integration via gRPC
- [ ] High-level planning and decision making

### Phase 3: Advanced Behaviors
- [ ] Complex multi-step task execution
- [ ] Learning from demonstration
- [ ] Adaptive behavior based on environment

## 🚦 Quick Start

> **Note**: For detailed installation and usage instructions, see [`USAGE.md`](docs/USAGE.md)

### Prerequisites
- Ubuntu 22.04 with ROS2 (Iron or Humble)
- Python 3.10+
- Unitree GO2 robot with network access

### Basic Setup
```bash
# Clone the repository
git clone --recurse-submodules https://github.com/your-username/shadowhound.git
cd shadowhound

# Install dependencies
sudo apt install ros-$ROS_DISTRO-image-tools ros-$ROS_DISTRO-vision-msgs
pip install -r requirements.txt

# Build the workspace
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build

# Launch the system
source install/setup.bash
export ROBOT_IP="your_robot_ip"
export CONN_TYPE="webrtc"
ros2 launch go2_robot_sdk robot.launch.py
```

## 📚 Documentation

- **[Usage Guide](docs/USAGE.md)**: Detailed setup and operation instructions
- **[Developer Guide](AGENTS.md)**: Coding standards and agent interaction patterns
- **[Project Context](docs/project_context.md)**: Technical architecture and design decisions
- **[Original SDK Docs](docs/original_readme_archived.md)**: Archived documentation from base SDK

## 🤝 Contributing

We welcome contributions! This project follows the coding standards and patterns outlined in [`AGENTS.md`](AGENTS.md). Key guidelines:

- **Language**: Python for ROS nodes, C++ only for performance-critical paths
- **Style**: `ruff` + `black` for Python, `ament_uncrustify` for C++
- **Testing**: `pytest` for Python, `ament_add_gtest` for C++
- **Documentation**: Clear docstrings and README updates

## 🐛 Known Issues

- **Joystick Control**: Logitech F710 compatibility issues on some systems (see [USAGE.md](docs/USAGE.md#joystick-troubleshooting))
- **Joint States**: 1Hz update rate with firmware v1.1.7 causes URDF lag
- **Initial Setup**: WebRTC connection may require closing mobile app first

## 🙏 Acknowledgments

This project builds upon the excellent work of:
- [@tfoldi](https://github.com/tfoldi) for [go2-webrtc](https://github.com/tfoldi/go2-webrtc)
- [@abizovnuralem](https://github.com/abizovnuralem) for [go2_ros2_sdk](https://github.com/abizovnuralem/go2_ros2_sdk)
- The ROS2 and robotics open source communities

## 📄 License

This project is licensed under the BSD 2-clause License - see the [LICENSE](LICENSE) file for details.

---

**Ready to build the future of embodied AI? Let's make robots that truly understand and interact with our world! 🤖✨**
