# WebRTC Configuration Guide

## Overview

The Unitree Go2 robot supports **two communication protocols** for different use cases:

1. **CycloneDDS (Ethernet)** - Direct motor control, sensors, basic navigation
2. **WebRTC (WiFi)** - Full API access including high-level behaviors (sit, stand, dance, wave)

**For DIMOS skills to work, you MUST use WebRTC mode.**

---

## Quick Reference

| Feature | CycloneDDS (Ethernet) | WebRTC (WiFi) |
|---------|----------------------|---------------|
| Connection | Ethernet cable | WiFi network |
| Sensors/Odometry | ✅ Yes | ✅ Yes |
| Camera | ✅ Yes | ✅ Yes |
| Direct motor control (`/cmd_vel`) | ✅ Yes | ✅ Yes |
| Teleop (gamepad) | ✅ Yes | ✅ Yes |
| High-level API (sit/stand/wave) | ❌ No | ✅ Yes |
| DIMOS skills | ❌ No | ✅ Yes |
| Configuration | Default | `CONN_TYPE=webrtc` |

---

## When to Use Each Mode

### Use CycloneDDS (Ethernet) for:
- Direct motor control (velocity commands)
- Sensor data collection (cameras, lidar, IMU)
- Teleop with gamepad
- Basic navigation without behaviors
- Development/testing without robot behaviors

### Use WebRTC (WiFi) for:
- **DIMOS autonomous behaviors** ← **YOU NEED THIS**
- High-level API commands (sit, stand, wave, dance)
- Mission execution with skills
- Full robot capabilities
- Production deployment

---

## Configuration Steps

### 1. Set CONN_TYPE Environment Variable

**In `.env` file:**
```bash
# Connection type for Unitree Go2 communication
CONN_TYPE=webrtc  # Required for DIMOS skills!
```

**Or in terminal:**
```bash
export CONN_TYPE=webrtc
```

### 2. Configure Robot WiFi Connection

The robot must be connected to your WiFi network for WebRTC mode to work.

#### Option A: Using Unitree App (Recommended)
1. Download "Unitree Go2" app on your phone
2. Connect phone to robot's hotspot (Go2_xxxxx)
3. Open app → Settings → WiFi Settings
4. Connect robot to your WiFi network
5. Note the WiFi IP address displayed (e.g., 192.168.1.103)

#### Option B: Using Web Interface
1. Connect to robot via Ethernet (192.168.123.161)
2. Open browser: http://192.168.123.161:8081
3. Navigate to Network Settings
4. Configure WiFi credentials
5. Note the assigned IP address

### 3. Update GO2_IP in .env

Update the robot IP address to match the WiFi IP:

```bash
# Robot IP address
# IMPORTANT: Use WiFi IP for WebRTC mode!
GO2_IP=192.168.1.103  # Replace with your robot's WiFi IP
```

**Note:** The WiFi IP is usually different from the Ethernet IP!
- Ethernet IP: 192.168.123.161 (fixed, via USB/Ethernet)
- WiFi IP: 192.168.1.xxx (DHCP, from your router)

### 4. Verify Network Connectivity

Test that you can reach the robot on WiFi:

```bash
# Ping robot's WiFi IP
ping 192.168.1.103  # Replace with your robot's WiFi IP

# Should see replies like:
# 64 bytes from 192.168.1.103: icmp_seq=1 ttl=64 time=5.2 ms
```

If ping fails:
- Check robot WiFi is connected (LED indicators)
- Verify IP address is correct (check in app)
- Ensure laptop and robot on same network
- Try rebooting robot

### 5. Launch with WebRTC Enabled

```bash
# Option 1: Using start.sh (recommended)
./start.sh  # Automatically uses CONN_TYPE from .env

# Option 2: Manual launch
export CONN_TYPE=webrtc
export GO2_IP=192.168.1.103
ros2 launch shadowhound_mission_agent bringup.launch.py
```

### 6. Verify WebRTC Connection

Check logs for confirmation:

```bash
# In mission agent logs, look for:
[INFO] Connection type: webrtc
[INFO] WebRTC mode: High-level API commands (sit, stand, wave) enabled

# Check that WebRTC topics exist:
ros2 topic list | grep webrtc
# Should see:
#   /webrtc_req
#   /webrtcreq
#   /xfk_webrtcreq
```

---

## Troubleshooting

### Problem: "Teleop works but robot doesn't respond to 'sit' command"

**Diagnosis:** You're in CycloneDDS mode (Ethernet).

**Solution:**
1. Set `CONN_TYPE=webrtc` in `.env`
2. Connect robot to WiFi
3. Update `GO2_IP` to WiFi IP
4. Restart system

**Explanation:** Teleop uses `/cmd_vel` (direct motor control) which works over CycloneDDS. High-level behaviors like "sit" require WebRTC API.

---

### Problem: "Cannot connect to robot in WebRTC mode"

**Diagnosis:** Network connectivity issue.

**Solution:**
1. Verify robot is on WiFi:
   ```bash
   # Check in Unitree app - should show WiFi connected
   ```

2. Verify IP address is correct:
   ```bash
   ping 192.168.1.103  # Your robot's WiFi IP
   ```

3. Check that you're using WiFi IP, not Ethernet IP:
   - ❌ 192.168.123.161 (Ethernet IP - won't work for WebRTC)
   - ✅ 192.168.1.103 (WiFi IP - correct for WebRTC)

4. Ensure laptop and robot on same network:
   ```bash
   # Check your laptop's network
   ip addr show
   # Should see IP in same subnet (e.g., 192.168.1.xxx)
   ```

---

### Problem: "Commands timeout after 30 seconds"

**Diagnosis:** WebRTC API not receiving commands.

**Solution:**
1. Check that bare DDS subscriber exists:
   ```bash
   ros2 topic info /webrtcreq --verbose
   # Should show subscriber: _CREATED_BY_BARE_DDS_APP_
   ```

2. If no subscriber, robot's WebRTC service may not be running:
   ```bash
   # Reboot robot
   # Wait for all services to start (~30 seconds after boot)
   ```

3. Try publishing directly to test:
   ```bash
   # Sit command (API 1009)
   ros2 topic pub --once /webrtc_req go2_interfaces/msg/WebRtcReq \
     "{id: 0, topic: 'rt/api/sport/request', api_id: 1009, parameter: '', priority: 0}"
   ```

---

### Problem: "WARN: CycloneDDS mode: High-level API commands NOT available"

**Diagnosis:** CONN_TYPE not set or set to cyclonedds.

**Solution:**
1. Check .env file:
   ```bash
   grep CONN_TYPE .env
   # Should show: CONN_TYPE=webrtc
   ```

2. If missing, add it:
   ```bash
   echo "CONN_TYPE=webrtc" >> .env
   ```

3. Restart the system:
   ```bash
   ./start.sh
   ```

---

## Advanced Configuration

### Using Both Modes Simultaneously

You can have TWO connections at the same time:
- **Ethernet (CycloneDDS)**: For sensors/odometry/camera
- **WiFi (WebRTC)**: For high-level API commands

This is the **recommended production setup** for maximum reliability.

**Configuration:**
```bash
# In .env
CONN_TYPE=webrtc  # Primary protocol for API commands
GO2_IP=192.168.1.103  # WiFi IP

# Robot should be connected via:
# 1. Ethernet cable (for sensor data)
# 2. WiFi network (for API commands)
```

### Switching Between Modes

To switch from WebRTC back to CycloneDDS (if needed):

```bash
# In .env, comment out or change CONN_TYPE
# CONN_TYPE=webrtc  # Commented out = uses cyclonedds default
CONN_TYPE=cyclonedds  # Explicit CycloneDDS mode

# Update IP to Ethernet IP
GO2_IP=192.168.123.161  # Ethernet IP

# Restart system
./start.sh
```

### Environment Variable Priority

Environment variables are loaded in this order (last wins):
1. Default value (cyclonedds)
2. `.env` file
3. Shell export `export CONN_TYPE=webrtc`
4. Launch parameter (if implemented)

---

## Testing WebRTC Connection

### Test 1: Direct API Command

```bash
# Source environment
source .shadowhound_env

# Sit command (API 1009)
ros2 topic pub --once /webrtc_req go2_interfaces/msg/WebRtcReq \
  "{id: 0, topic: 'rt/api/sport/request', api_id: 1009, parameter: '', priority: 0}"

# Robot should sit down within 2 seconds
```

### Test 2: Stand Command

```bash
# Stand command (API 1001)
ros2 topic pub --once /webrtc_req go2_interfaces/msg/WebRtcReq \
  "{id: 0, topic: 'rt/api/sport/request', api_id: 1001, parameter: '', priority: 0}"

# Robot should stand up
```

### Test 3: Wave Command

```bash
# Wave command (API 1016)
ros2 topic pub --once /webrtc_req go2_interfaces/msg/WebRtcReq \
  "{id: 0, topic: 'rt/api/sport/request', api_id: 1016, parameter: '', priority: 0}"

# Robot should wave with front paw
```

### Test 4: Web Interface

```bash
# Open web dashboard
firefox http://localhost:8080

# Try commands:
- "sit down"
- "stand up"  
- "wave your paw"

# Robot should execute behaviors
```

---

## API Command Reference

Common API IDs for testing:

| API ID | Command | Description |
|--------|---------|-------------|
| 1001 | Stand | Default standing pose |
| 1002 | Walk | Start walking |
| 1009 | Sit | Sit down |
| 1011 | Hello | Wave with front paw |
| 1016 | Wave | Alternative wave gesture |
| 1019 | Dance | Perform dance routine |
| 1101 | Stop | Emergency stop |

See DIMOS documentation for full API reference.

---

## Network Architecture

```
┌─────────────────────────────────────────────────────────────┐
│ Laptop (192.168.1.100)                                      │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ROS2 Nodes:                                               │
│  ┌──────────────────┐         ┌─────────────────────┐     │
│  │ mission_agent    │────────▶│ go2_driver_node     │     │
│  │ (DIMOS)          │ /webrtc_│ (go2_ros2_sdk)      │     │
│  └──────────────────┘    req  └─────────────────────┘     │
│         │                              │                   │
│         │ CONN_TYPE=webrtc             │ CONN_TYPE=webrtc  │
│         ▼                              ▼                   │
└─────────────────────────────────────────────────────────────┘
         │                              │
         │ WiFi (192.168.1.x)           │
         ▼                              ▼
┌─────────────────────────────────────────────────────────────┐
│ Robot (192.168.1.103 WiFi, 192.168.123.161 Ethernet)       │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  Bare DDS App:          ROS2 Bridge:                        │
│  ┌──────────────────┐   ┌───────────────────┐             │
│  │ /webrtcreq       │   │ Sensors, Odometry │             │
│  │ (WebRTC API)     │   │ Camera, Motors    │             │
│  └──────────────────┘   └───────────────────┘             │
│         │                        │                         │
│         ▼                        ▼                         │
│  ┌─────────────────────────────────────────┐              │
│  │ Sport Mode Controller                   │              │
│  │ (sit, stand, dance, wave, etc.)         │              │
│  └─────────────────────────────────────────┘              │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

**Key Points:**
- `CONN_TYPE` controls which pathway is used
- `webrtc` enables the Bare DDS App connection
- `cyclonedds` only uses ROS2 Bridge (no high-level behaviors)
- Both sensor data and API commands can coexist (dual connection)

---

## Reference Documentation

- **Architecture Deep Dive:** `docs/WEBRTC_DISCOVERY.md`
- **Debugging Guide:** `docs/DEBUGGING_ROBOT_COMMANDS.md`
- **DIMOS Documentation:** `src/dimos-unitree/README.md`
- **Go2 SDK:** `external/go2_ros2_sdk/README.md`

---

## Summary Checklist

Before launching with WebRTC mode:

- [ ] `CONN_TYPE=webrtc` set in `.env`
- [ ] Robot connected to WiFi network
- [ ] `GO2_IP` set to robot's **WiFi IP** (not Ethernet IP)
- [ ] Can ping robot's WiFi IP from laptop
- [ ] Laptop and robot on same network/subnet
- [ ] Robot fully booted (wait 30s after power on)
- [ ] `.env` file loaded (`source .env` or via `start.sh`)

**If all checked, you're ready for DIMOS autonomous behaviors! 🎉**

---

**For issues not covered here, see:**
- `docs/WEBRTC_DISCOVERY.md` - Complete investigation documentation
- `docs/DEBUGGING_ROBOT_COMMANDS.md` - Step-by-step debugging guide
- ShadowHound Issues: https://github.com/your-org/shadowhound/issues
