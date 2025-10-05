# WebRTC vs CycloneDDS Discovery - Robot Commands Not Executing

## Date
October 5, 2025

## Problem Statement
Robot commands from DIMOS (sit, stand, wave, etc.) were being queued and "executed" but the physical robot was not responding. However, teleop (gamepad control) worked perfectly.

## Investigation Summary

### What We Discovered

The Unitree Go2 ROS2 SDK supports **two separate communication protocols**:

1. **CycloneDDS (Ethernet)** - Default when `CONN_TYPE` not set
   - Handles: Sensors, odometry, camera, IMU, lidar
   - Handles: **Direct motor control** via `/cmd_vel`
   - ✅ **Working perfectly** - This is why teleop works!

2. **WebRTC (WiFi)**
   - Handles: **High-level API commands** (sit, stand, dance, wave, etc.)
   - Requires: WiFi connection to robot
   - ❌ **Was not enabled** - This is why DIMOS skills don't work!

### Key Findings

#### Topic Analysis
```bash
# Topics discovered:
/webrtc_req          # ROS2 topic (go2_interfaces/msg/WebRtcReq)
                     # Subscribed by: /go2_driver_node
                     # Used by: DIMOS for API commands
                     
/webrtcreq           # Bare DDS topic (std_msgs/msg/String) 
                     # Subscribed by: _CREATED_BY_BARE_DDS_APP_ (on robot)
                     # This is the actual WebRTC connection to robot hardware!
                     
/xfk_webrtcreq       # Alternate bare DDS topic (std_msgs/msg/String)
                     # Also subscribed by bare DDS app on robot
```

#### Connection Architecture
```
┌─────────────────────────────────────────────────────────────┐
│ ROS2 Laptop                                                 │
│                                                             │
│  ┌──────────────┐         ┌──────────────┐                │
│  │   Teleop     │─/cmd_vel→│ go2_driver  │                │
│  │  (gamepad)   │         │    _node     │                │
│  └──────────────┘         │              │                │
│                           │  CycloneDDS  │◄───Ethernet────┤───► Robot
│  ┌──────────────┐         │  (working!)  │                │     Sensors
│  │    DIMOS     │─/webrtc→│              │                │     Motors ✅
│  │   Skills     │  _req   └──────────────┘                │
│  └──────────────┘              ↓                           │
│                           Not forwarded                    │
│                           to WebRTC!                       │
│                                                             │
│  ┌──────────────────────────────────────────────────────┐ │
│  │ Bare DDS WebRTC App (on robot)                       │ │
│  │ Subscribes to: /webrtcreq, /xfk_webrtcreq           │◄┼─── WiFi
│  │ (QoS mismatch prevents ROS2 from publishing)        │ │    (not used)
│  └──────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

### Why Commands Didn't Work

1. **DIMOS publishes** to `/webrtc_req` (go2_interfaces/msg/WebRtcReq)
2. **go2_driver_node receives** the command but doesn't forward it to robot hardware
3. **Robot's WebRTC app** listens on `/webrtcreq` (different topic, different type!)
4. **QoS mismatch** prevents ROS2 publishers from connecting to bare DDS subscriber
5. **Result**: Commands queued, logged, but never reach robot hardware

### Why Teleop Worked

Teleop publishes to `/cmd_vel` which:
- Goes through go2_driver_node (already connected via CycloneDDS/Ethernet)
- Uses direct motor control (low-level, always available)
- Doesn't require WebRTC API commands

## Diagnostic Commands Used

```bash
# Check robot mode
ros2 topic echo /go2_states --once | grep -E "(mode|progress)"

# Check topic subscribers
ros2 node info /go2_driver_node | grep -A20 "Subscribers"

# Discover WebRTC topics
ros2 topic list | grep -i webrtc

# Check topic details
ros2 topic info /webrtc_req --verbose
ros2 topic info /webrtcreq --verbose

# Test direct command
ros2 topic pub --once /webrtc_req go2_interfaces/msg/WebRtcReq "{
  id: 0,
  topic: 'rt/api/sport/request',
  api_id: 1009,
  parameter: '',
  priority: 0
}"
```

## Solution: Enable WebRTC Mode

### Requirements
1. Robot must be on **WiFi network** (not Ethernet)
2. Set `CONN_TYPE=webrtc` environment variable
3. Restart entire stack with WebRTC enabled

### Configuration Changes Needed

#### 1. Environment Variable
```bash
export CONN_TYPE=webrtc
```

#### 2. Network Connection
- Connect robot to WiFi via Unitree app
- Disconnect Ethernet cable (or configure to use WiFi for WebRTC)
- Note robot's WiFi IP address

#### 3. Launch Configuration
Ensure launch files respect `CONN_TYPE` environment variable and configure driver accordingly.

## Testing After WebRTC Enabled

### Verify WebRTC Connection
```bash
# Check if WebRTC app is accessible
ros2 topic info /webrtcreq --verbose
# Should show publisher from ROS2 connecting successfully

# Test sit command
ros2 topic pub --once /webrtc_req go2_interfaces/msg/WebRtcReq "{
  id: 0,
  topic: 'rt/api/sport/request', 
  api_id: 1009,
  parameter: '',
  priority: 0
}"

# Robot should physically sit down!
```

### Verify Web UI Commands
```bash
# Start the mission agent (should already be running)
# Open http://localhost:8080
# Try "sit down" command
# Robot should respond!
```

## Files Modified

### ShadowHound Code (Our Repository)
- `mission_agent.py` - Set `webrtc_api_topic='webrtc_req'` 
- `scripts/start.sh` - Need to add `export CONN_TYPE=webrtc`
- `scripts/test_robot_command.sh` - Updated to use `/webrtc_req`

### DIMOS Submodule (Do Not Modify)
- `unitree_ros_control.py` - Has mode detection logic (uses `progress` field)
  - Note: We discovered mode detection was too strict, but should not modify submodule

## Lessons Learned

1. **Always check connection type** when debugging robot commands
2. **CycloneDDS ≠ WebRTC** - Different protocols, different capabilities
3. **Teleop working doesn't mean API commands work** - Different pathways
4. **Bare DDS apps** can exist outside ROS2 node graph
5. **Topic names matter** - `/webrtc_req` vs `/webrtcreq` are different!
6. **QoS profiles must match** between publishers and subscribers

## Related Issues

- Robot mode=9 (sport mode) with progress=0 (idle) - This is normal!
- DIMOS mode detection logic assumes mode must be 1 for IDLE (too strict)
- Command queue waits for progress=0 (correct) but also checks mode=1 (wrong for sport mode)

## Next Steps

1. ✅ Document discovery (this file)
2. ⏳ Configure system to use WebRTC
3. ⏳ Test all DIMOS skills work with WebRTC
4. ⏳ Update documentation for future deployments
5. ⏳ Consider creating fallback: try WebRTC, fallback to velocity control

## References

- Go2 ROS2 SDK: https://github.com/abizovnuralem/go2_ros2_sdk
- WebRTC Topic Interface: See SDK README section
- Unitree Go2 API IDs: See `robot_commands.py` in SDK

## Credits

Investigation by: GitHub Copilot + Daniel Martinez
Date: October 5, 2025
Session: DIMOS Integration Debug Marathon
