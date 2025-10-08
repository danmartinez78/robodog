# ShadowHound Deployment Topology

**IMPORTANT**: Read this file before making assumptions about the runtime environment!

## Environment Overview

### Desktop (Devcontainer) - Development
- **Location**: `/workspaces/shadowhound` (devcontainer)
- **Purpose**: Code development, testing, commits, git operations
- **User**: GitHub Copilot agent works here
- **ROS**: May or may not have ROS topics running
- **Robot**: No robot connected
- **Usage**: Edit code, run builds, create documentation

### Laptop (Ubuntu 22.04) - Deployment
- **Location**: `~/shadowhound` (daniel@9510)
- **Purpose**: Runtime environment with robot connection
- **User**: daniel (human operator)
- **ROS**: Full ROS2 system running with robot
- **Robot**: Unitree Go2 connected (IP: 192.168.1.103)
- **Usage**: Deploy and run code, test with actual robot

## Deployment Workflow

```
Desktop (Devcontainer)              Laptop (daniel@9510)
┌─────────────────────┐            ┌─────────────────────┐
│                     │            │                     │
│  1. Edit code       │            │  3. Pull changes    │
│  2. Test builds     │    Git     │  4. Rebuild         │
│  3. Commit & push   │  ────────> │  5. Restart nodes   │
│                     │  (GitHub)  │  6. Test with robot │
│                     │            │                     │
└─────────────────────┘            └─────────────────────┘
         ↑                                    │
         │                                    │
         └──────── Feedback / Issues ─────────┘
```

## Key Differences

| Aspect | Desktop (Dev) | Laptop (Deploy) |
|--------|---------------|-----------------|
| ROS Topics | Minimal/None | Full robot topics |
| Camera Feed | No | Yes (`/camera/image_raw`, `/camera/compressed`) |
| Robot Connection | No | Yes (via WiFi/Ethernet) |
| Web UI | Can test locally | Primary interface |
| Git Operations | Yes (push code) | Yes (pull code) |
| Purpose | Development | Runtime |

## Available Topics on Laptop

### Critical Topics for Mission Agent
- `/camera/image_raw` - Raw camera feed (~14Hz)
- `/camera/compressed` - Compressed camera feed (~14Hz)
- `/camera/camera_info` - Camera calibration info
- `/odom` - Odometry (robot position/velocity)
- `/imu` - IMU data (acceleration/gyro)
- `/joint_states` - Robot joint positions
- `/go2_states` - Robot state (STAND, SIT, etc.)
- `/cmd_vel` - Velocity commands to robot
- `/mission_command` - Mission agent commands
- `/mission_status` - Mission agent status
- `/webrtc_req` - WebRTC API for DIMOS high-level commands

### Navigation Topics
- `/map` - SLAM map
- `/scan` - LiDAR scan
- `/local_costmap/*` - Local navigation costmap
- `/global_costmap/*` - Global navigation costmap
- `/plan` - Navigation plan
- Various Nav2 topics

## Common Pitfalls

### ❌ DON'T:
- Assume topics exist in devcontainer (they usually don't)
- Run `ros2 topic list` in devcontainer and expect robot topics
- Test camera feed in devcontainer (no camera connected)
- Forget to rebuild on laptop after pulling changes

### ✅ DO:
- Edit code in devcontainer, test deployment on laptop
- Ask user to verify topics on laptop before debugging
- Remember that laptop has the actual robot connection
- Provide deployment instructions for laptop after code changes

## Debugging Checklist

When user reports an issue:

1. **Clarify location**: "Are you testing on laptop or desktop?"
2. **Check deployment**: "Did you pull latest code and rebuild?"
3. **Verify topics**: Ask user to run `ros2 topic list | grep <topic>`
4. **Check nodes**: Ask user to run `ros2 node list`
5. **Check logs**: Ask for node output or errors
6. **Browser cache**: Remind to hard-refresh UI (`Ctrl+Shift+R`)

## Current Issue Pattern

**User reports**: "Camera feed not working"

**Agent should ask**:
1. ✅ Are you on laptop? (YES - deployment environment)
2. ✅ Did you pull latest code? 
3. ✅ Did you rebuild? `colcon build --packages-select shadowhound_mission_agent`
4. ✅ Did you restart the node?
5. ✅ Did you hard-refresh browser? `Ctrl+Shift+R`
6. ✅ Are camera topics available? `ros2 topic list | grep camera` (YES - confirmed)
7. ❓ Is mission agent node running? `ros2 node list | grep mission`
8. ❓ Are there errors in node output?
9. ❓ Is web interface accessible? `http://localhost:8080`
10. ❓ What does browser console show? (F12 → Console tab)

## Notes

- User (daniel) is experienced with ROS2 and Linux
- Robot is Unitree Go2 quadruped
- Using DIMOS framework for high-level robot control
- Web UI runs on port 8080 by default
- Camera feed should be ~14Hz from robot

---

**Last Updated**: October 8, 2025  
**Maintainer**: Development team  
**Purpose**: Help AI assistant remember deployment topology
