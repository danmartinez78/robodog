# ShadowHound Setup Guide for Ubuntu Laptop

**Target**: Real Unitree Go2 Robot  
**Environment**: Ubuntu 22.04 (native or WSL2)  
**Date**: October 4, 2025

---

## Prerequisites

### System Requirements
- Ubuntu 22.04 LTS
- ROS2 Humble installed
- Git, Python 3.10+
- Network access to Unitree Go2 robot

### Network Setup
The Go2 robot creates a WiFi network you need to connect to:
- **SSID**: Unitree_Go2XXXXXX (check robot's body)
- **Password**: 00000000 (8 zeros)
- **Robot IP**: 192.168.12.1 (default)

---

## Installation Steps

### 1. Clone Workspace

```bash
cd ~
git clone https://github.com/danmartinez78/shadowhound.git
cd shadowhound
git checkout feature/dimos-integration
```

### 2. Install ROS2 Dependencies

```bash
# Install vcstool
sudo apt install python3-vcstool python3-rosdep

# Initialize rosdep (if not done)
sudo rosdep init
rosdep update
```

### 3. Import Repositories

The `shadowhound.repos` file is already configured to use your forks:

```bash
vcs import src < shadowhound.repos
```

**Important**: This will import `dimos-unitree` at commit `3b0122e`, which already has the `go2_ros2_sdk` submodule pointing to your `robodan_dev` branch. **No git changes should appear** after this step!

### 4. Initialize Submodules

```bash
cd src/dimos-unitree
git submodule update --init --recursive
```

Verify go2_ros2_sdk submodule:
```bash
cd dimos/robot/unitree/external/go2_ros2_sdk
git remote -v  # Should show danmartinez78/go2_ros2_sdk
git log -1     # Should be on robodan_dev
```

### 5. Install Python Dependencies

```bash
cd ~/shadowhound

# Install PyTorch (CPU version for laptop)
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu

# Install go2_ros2_sdk requirements
pip3 install -r src/dimos-unitree/dimos/robot/unitree/external/go2_ros2_sdk/requirements.txt

# Install ROS system dependencies
sudo apt install ros-humble-foxglove-bridge ros-humble-twist-mux \
    ros-humble-pointcloud-to-laserscan ros-humble-image-tools \
    ros-humble-vision-msgs clang portaudio19-dev

# Install remaining dependencies via rosdep
rosdep install --from-paths src --ignore-src -r -y
```

### 6. Build Workspace

```bash
cd ~/shadowhound

# Build (skipping problematic perception packages)
colcon build --symlink-install --packages-select \
    shadowhound_mission_agent shadowhound_bringup

# Source workspace
source install/setup.bash
```

---

## Configuration for Real Robot

### 1. Set Environment Variables

Create `~/shadowhound/robot.env`:

```bash
# Robot connection
export GO2_IP="192.168.12.1"          # Default Go2 IP
export ROS_DOMAIN_ID=42               # Isolated ROS network

# Agent configuration  
export OPENAI_API_KEY="sk-..."       # Your OpenAI API key
export AGENT_BACKEND="cloud"          # or "local"

# Network interface (adjust for your laptop)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Load it:
```bash
source ~/shadowhound/robot.env
source ~/shadowhound/install/setup.bash
```

### 2. Connect to Robot

1. **Turn on Go2 robot** (press power button on battery)
2. **Wait for WiFi** (Go2 creates access point)
3. **Connect laptop** to Go2's WiFi network
4. **Verify connection**:
   ```bash
   ping 192.168.12.1
   ```

### 3. Test Connection

```bash
# In terminal 1: Monitor Go2 topics
ros2 topic list

# You should see topics like:
# /go2/state
# /go2/cmd_vel
# /camera/image_raw
# etc.
```

---

## Running ShadowHound

### Launch System (Real Robot)

```bash
# Source environment
source ~/shadowhound/robot.env
source ~/shadowhound/install/setup.bash

# Launch with real robot
ros2 launch shadowhound_bringup shadowhound.launch.py mock_robot:=false
```

### Send Test Commands

```bash
# In another terminal
source ~/shadowhound/install/setup.bash

# Simple command
ros2 topic pub /mission_command std_msgs/String "data: 'stand up'"

# Monitor status
ros2 topic echo /mission_status
```

### Test Sequence

```bash
# 1. Stand up
ros2 topic pub --once /mission_command std_msgs/String "data: 'stand up'"

# 2. Hello gesture  
ros2 topic pub --once /mission_command std_msgs/String "data: 'wave hello'"

# 3. Sit down
ros2 topic pub --once /mission_command std_msgs/String "data: 'sit down'"
```

---

## Troubleshooting

### Issue: Git shows dimos-unitree as modified

**Cause**: Wrong commit checked out  
**Solution**:
```bash
cd ~/shadowhound/src/dimos-unitree
git checkout 3b0122eec621d32f31d0c32e6565f4614e7de1f9
git submodule update --init --recursive
```

### Issue: Can't connect to robot

**Check**:
1. Robot is powered on (green LED)
2. Connected to Go2 WiFi network
3. Can ping 192.168.12.1
4. Firewall not blocking ROS traffic

### Issue: "DIMOS not available" error

**Solution**:
```bash
# Add dimos to PYTHONPATH
export PYTHONPATH="${HOME}/shadowhound/src/dimos-unitree:${PYTHONPATH}"
source ~/shadowhound/install/setup.bash
```

### Issue: Build errors with CUDA

**Expected**: Deformable-DETR requires CUDA (perception model)  
**Solution**: We skip it - not needed for basic operation

---

## Safety Notes

⚠️ **IMPORTANT SAFETY PRECAUTIONS**:

1. **Clear Space**: Ensure 2m radius is clear before commanding robot
2. **Emergency Stop**: Keep hand on power button for emergency shutdown
3. **Start Simple**: Begin with `stand up` and `sit down` commands
4. **Monitor Status**: Always watch `/mission_status` topic
5. **Kill Switch**: E-stop button is on the robot's back

### Emergency Procedures

**Robot misbehaving**:
1. Press power button to shut down
2. Or publish emergency stop:
   ```bash
   ros2 topic pub /go2/cmd_vel geometry_msgs/Twist '{linear: {x: 0}, angular: {z: 0}}'
   ```

---

## Next Steps

Once basic operation is verified:

1. ✅ Test basic skills (stand, sit, walk)
2. 📹 Enable video streaming (set `disable_video_stream: false`)
3. 🗺️ Test navigation commands
4. 🧠 Try planning agent for complex missions
5. 🎯 Create custom mission scripts

---

## Verification Checklist

Before testing with robot:

- [ ] Workspace built successfully
- [ ] No git changes in dimos-unitree
- [ ] Robot is accessible (ping works)
- [ ] OpenAI API key configured
- [ ] Safety area is clear
- [ ] Emergency stop plan in place

---

## Quick Reference

```bash
# Setup
source ~/shadowhound/robot.env
source ~/shadowhound/install/setup.bash

# Launch
ros2 launch shadowhound_bringup shadowhound.launch.py mock_robot:=false

# Test command
ros2 topic pub --once /mission_command std_msgs/String "data: 'stand up'"

# Monitor
ros2 topic echo /mission_status
```

---

## Support

Issues? Check:
1. This guide's troubleshooting section
2. `docs/INTEGRATION_STATUS.md` for architecture
3. Package READMEs in `src/shadowhound_*/`
4. DIMOS documentation: https://github.com/dimensionalOS/dimos-unitree

**Good luck and stay safe! 🐕🤖**
