# Direct WebRTC Test Guide

## Purpose

Test the go2_ros2_sdk driver in WebRTC mode **without DIMOS** to validate:
1. ✅ WebRTC connection works over WiFi
2. ✅ Commands reach robot hardware
3. ✅ Robot physically responds to API commands

This isolates the SDK layer to confirm WebRTC is working before testing DIMOS integration.

---

## Prerequisites

### 1. Robot WiFi Setup
```bash
# Robot MUST be on WiFi network (not just Ethernet!)
# Use Unitree app:
#   1. Connect phone to robot's hotspot OR robot to your WiFi
#   2. Open Unitree app → Settings → Network Settings
#   3. Note the WiFi IP address (e.g., 192.168.1.103)
```

### 2. Update GO2_IP
```bash
# Edit .env file
nano .env

# Set WiFi IP (NOT Ethernet IP!)
GO2_IP=192.168.1.103  # ← Replace with YOUR robot's WiFi IP
CONN_TYPE=webrtc       # ← Make sure this is set

# Note: The SDK internally uses ROBOT_IP, but our scripts automatically
# export ROBOT_IP=$GO2_IP for compatibility. You only need to set GO2_IP.
```

### 3. Verify Network
```bash
# Test connectivity
ping 192.168.1.103  # Should get replies

# Check if robot is on network
nmap -sn 192.168.1.0/24 | grep -i unitree  # Optional: scan network
```

---

## Running the Test

### Terminal 1: Launch SDK Driver
```bash
cd /workspaces/shadowhound

# Run test script (launches driver in WebRTC mode)
./scripts/test_webrtc_direct.sh
```

**Expected Output:**
```
╔════════════════════════════════════════════════════════════════╗
║  Direct WebRTC Test - go2_ros2_sdk Only                       ║
╚════════════════════════════════════════════════════════════════╝

Configuration:
  • CONN_TYPE: webrtc
  • GO2_IP: 192.168.1.103
  • ROBOT_IP: 192.168.1.103

[1/4] Verifying robot connectivity...
✓ Robot reachable at 192.168.1.103

[2/4] Locating go2_ros2_sdk launch file...
✓ Found: src/dimos-unitree/dimos/robot/unitree/external/go2_ros2_sdk/launch/robot.launch.py

[3/4] Launching go2_ros2_sdk driver (WebRTC mode)...
✓ Driver launched (PID: 12345)

[4/4] Waiting for topics to appear...
✓ Robot topics detected!

Available topics:
  /cmd_vel_out
  /go2_states
  /webrtc_req
  /camera/image_raw
  ...

Driver is running in foreground. Press Ctrl+C to stop.
=== Live Logs (last 50 lines) ===
```

**Look for WebRTC indicators in logs:**
- ✅ "WebRTC" mentioned
- ✅ "LocalSTA" or "LocalAP" connection mode
- ✅ No errors about "connection refused"

---

### Terminal 2: Test Commands

Open a **second terminal** and source the environment:
```bash
cd /workspaces/shadowhound
source .shadowhound_env
```

#### Test 1: Check Robot State
```bash
ros2 topic echo /go2_states --once
```

**Expected:** Should see robot state data (mode, progress, etc.)

---

#### Test 2: Sit Command (API 1009)
```bash
ros2 topic pub --once /webrtc_req go2_interfaces/msg/WebRtcReq \
  "{id: 0, topic: 'rt/api/sport/request', api_id: 1009, parameter: '', priority: 0}"
```

**Expected:** 
- ✅ Command published successfully
- ✅ **Robot physically sits down!** 🐕⬇️

**If robot doesn't move:**
- Check Terminal 1 logs for errors
- Verify CONN_TYPE=webrtc in environment
- Check robot WiFi connection

---

#### Test 3: Stand Command (API 1001)
```bash
ros2 topic pub --once /webrtc_req go2_interfaces/msg/WebRtcReq \
  "{id: 0, topic: 'rt/api/sport/request', api_id: 1001, parameter: '', priority: 0}"
```

**Expected:** ✅ **Robot stands up!** 🐕

---

#### Test 4: Wave Command (API 1021)
```bash
ros2 topic pub --once /webrtc_req go2_interfaces/msg/WebRtcReq \
  "{id: 0, topic: 'rt/api/sport/request', api_id: 1021, parameter: '', priority: 0}"
```

**Expected:** ✅ **Robot waves paw!** 🐕👋

---

#### Test 5: Check WebRTC Endpoint (Bare DDS)
```bash
ros2 topic info /webrtcreq --verbose
```

**Expected Output:**
```
Node name: _CREATED_BY_BARE_DDS_APP_
Node namespace: _CREATED_BY_BARE_DDS_APP_
Topic type: std_msgs/msg/String
...
Subscription count: 1
```

This confirms the **bare DDS application on the robot** is listening for WebRTC commands!

---

## Interpreting Results

### ✅ SUCCESS Indicators
1. **Robot responds physically** to sit/stand/wave commands
2. `/webrtcreq` topic shows `_CREATED_BY_BARE_DDS_APP_` subscriber
3. No "connection refused" or "timeout" errors
4. Driver logs show WebRTC-related messages

**If all above true:** ✅ **WebRTC is working!** Problem is in DIMOS integration layer.

---

### ❌ FAILURE Indicators

#### Robot doesn't move, but command published successfully
```
Possible causes:
1. CONN_TYPE not set (still using CycloneDDS)
2. Robot not on WiFi (only Ethernet connected)
3. GO2_IP points to Ethernet IP instead of WiFi IP
4. WebRTC service not running on robot
```

**Fix:**
```bash
# Verify environment
echo $CONN_TYPE  # Should be "webrtc"
echo $GO2_IP     # Should be WiFi IP

# Re-export if needed
export CONN_TYPE=webrtc
export GO2_IP=192.168.1.103

# Restart driver
./scripts/test_webrtc_direct.sh
```

---

#### No `/webrtcreq` topic with bare DDS subscriber
```
This means WebRTC endpoint on robot is not active!

Possible causes:
1. Robot not on WiFi
2. WebRTC service disabled on robot
3. Robot firmware doesn't support WebRTC
```

**Fix:**
- Verify robot is on WiFi (check Unitree app)
- Try restarting robot
- Check robot firmware version (WebRTC requires newer firmware)

---

#### Topics never appear (timeout)
```
Possible causes:
1. Robot not reachable (network issue)
2. Wrong IP address
3. Firewall blocking ROS2 DDS traffic
```

**Fix:**
```bash
# Test basic connectivity
ping $GO2_IP

# Check ROS_DOMAIN_ID (should be 0)
echo $ROS_DOMAIN_ID

# Try different RMW implementation
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

---

## Common API Commands Reference

| Command | API ID | Parameter | Description |
|---------|--------|-----------|-------------|
| **Stand** | 1001 | '' | Stand in normal pose |
| **Sit** | 1009 | '' | Sit down |
| **Wave** | 1021 | '' | Wave front paw |
| **Stretch** | 1023 | '' | Stretch pose |
| **Damping** | 1004 | '' | Enter damping mode |
| **Recovery Stand** | 1005 | '' | Stand up from lying down |
| **Hello** | 1023 | '' | Greeting gesture |

All use same topic format:
```bash
ros2 topic pub --once /webrtc_req go2_interfaces/msg/WebRtcReq \
  "{id: 0, topic: 'rt/api/sport/request', api_id: XXXX, parameter: '', priority: 0}"
```

---

## Next Steps After Testing

### If WebRTC Test SUCCEEDS ✅
**Conclusion:** SDK WebRTC works! Issue is in DIMOS integration.

**Actions:**
1. ✅ Confirm CONN_TYPE environment variable is passed to DIMOS
2. ✅ Check DIMOS robot initialization respects CONN_TYPE
3. ✅ Verify mission_agent passes environment correctly
4. ✅ Test DIMOS skills with WebRTC validated

### If WebRTC Test FAILS ❌
**Conclusion:** WebRTC connectivity issue at SDK level.

**Actions:**
1. ❌ Fix robot WiFi connection
2. ❌ Update GO2_IP to correct WiFi address
3. ❌ Verify CONN_TYPE=webrtc in environment
4. ❌ Check robot firmware supports WebRTC
5. ❌ Re-run test until WebRTC works

---

## Cleanup

When done testing:

```bash
# Terminal 1: Press Ctrl+C to stop driver

# Clean up any remaining processes
pkill -f go2_driver_node
pkill -f robot.launch

# Check logs if needed
ls -lh /tmp/webrtc_test_*.log
```

---

## Troubleshooting Quick Reference

### "Cannot reach robot"
```bash
# Check network
ip addr show  # Verify you're on same network as robot
ping $GO2_IP  # Test connectivity

# Check robot WiFi (use Unitree app)
# Robot IP: Settings → Network Settings → WiFi IP
```

### "Topics never appear"
```bash
# Check ROS2 environment
ros2 doctor --report  # Diagnose ROS2 setup

# Try manual topic list
ros2 topic list

# Check if driver is running
ps aux | grep go2_driver
```

### "Command published but robot doesn't move"
```bash
# Verify CONN_TYPE
echo $CONN_TYPE  # Must be "webrtc"

# Check if WebRTC endpoint exists
ros2 topic info /webrtcreq --verbose
# Should show: _CREATED_BY_BARE_DDS_APP_

# Monitor logs for errors
tail -f /tmp/webrtc_test_*.log
```

### "Permission denied" or "Cannot find launch file"
```bash
# Ensure workspace is built
cd /workspaces/shadowhound
colcon build --packages-select go2_robot_sdk go2_interfaces --symlink-install

# Source workspace
source install/setup.bash
```

---

## Success Criteria Checklist

Before considering test successful, verify:

- [ ] Driver launches without errors
- [ ] Robot topics appear (`/go2_states`, `/webrtc_req`, etc.)
- [ ] `ros2 topic echo /go2_states` shows live data
- [ ] `/webrtcreq` topic has `_CREATED_BY_BARE_DDS_APP_` subscriber
- [ ] Sit command makes robot **physically sit**
- [ ] Stand command makes robot **physically stand**
- [ ] Wave command makes robot **physically wave**
- [ ] No timeout or connection errors in logs

**If all checked:** ✅ **WebRTC is CONFIRMED WORKING!**

---

## Additional Diagnostics

### Check DDS Discovery
```bash
# See all DDS participants (should include robot)
ros2 daemon stop
ros2 daemon start
ros2 topic list -v
```

### Monitor Network Traffic
```bash
# Install if needed: sudo apt install tcpdump
sudo tcpdump -i any port 7400 -c 20  # DDS discovery
sudo tcpdump -i any port 9991        # WebRTC server (if custom port)
```

### Verify Environment
```bash
# Should all be set
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "GO2_IP: $GO2_IP"
echo "CONN_TYPE: $CONN_TYPE"
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
```

---

## References

- **WebRTC Discovery Investigation**: `docs/WEBRTC_DISCOVERY.md`
- **WebRTC Configuration Guide**: `docs/WEBRTC_CONFIGURATION.md`
- **General Debugging**: `docs/DEBUGGING_ROBOT_COMMANDS.md`
- **Test Script**: `scripts/test_webrtc_direct.sh`

---

**Good luck with your test! 🚀**
