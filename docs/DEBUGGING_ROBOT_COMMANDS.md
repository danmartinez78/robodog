# Debugging Robot Commands Not Executing

## Issue
Commands are being queued and sent via WebRTC but the robot is not physically responding.

## What's Working ✅
- Web UI sends commands successfully
- Agent receives and processes commands
- Skills are called (Sit, Hello/Wave, etc.)
- WebRTC requests are queued: `Queued WebRTC request: 1009`
- Commands are published to topic: `Sent WebRTC request: api_id=1009, topic=rt/api/sport/request`

## What's NOT Working ❌
- Physical robot does not move/respond
- Commands timeout after 30 seconds: `Robot has been busy for 30.1s, forcing queue to continue`

## Diagnostic Steps

### 1. Check if robot driver is running
```bash
ros2 node list | grep go2
```
Expected: `/go2_driver_node` or similar

### 2. Check if WebRTC topic exists and has subscribers
```bash
ros2 topic info /rt/api/sport/request
```
Expected: Should show publishers and subscribers

### 3. Check if commands are actually being published
```bash
ros2 topic echo /rt/api/sport/request
```
Try sending a command from web UI and see if messages appear

### 4. Check robot state topic
```bash
ros2 topic echo /go2_states --once
```
This confirms the robot driver is connected to the physical robot

### 5. Try direct WebRTC command (bypass DIMOS)
```bash
# Stand up command
ros2 topic pub --once /rt/api/sport/request std_msgs/msg/String "{data: '{\"header\":{\"identity\":{\"id\":123,\"api_id\":1001}}}'}"
```

### 6. Check network connectivity
```bash
ping $GO2_IP  # Should be 192.168.10.167
```

### 7. Check robot mode
The robot needs to be in the correct mode (not damping mode) for WebRTC commands to work.

## Common Causes

### 1. Robot not in correct mode
- Robot must be in "normal" mode, not damping mode
- WebRTC API requires specific robot state
- Solution: Check robot mode, may need to use joystick to enable

### 2. Wrong topic name
- Check if topic is `/rt/api/sport/request` vs `rt/api/sport/request` (leading slash)
- Solution: Verify topic name matches what driver expects

### 3. Message format incorrect
- WebRTC messages need specific JSON format
- Solution: Check message structure matches Go2 SDK expectations

### 4. Network issues
- Robot IP might be wrong or unreachable
- Robot on different subnet
- Solution: Verify `GO2_IP` environment variable, test ping

### 5. Driver not running
- Robot driver node may have crashed or not started
- Solution: Check `ros2 node list`, restart if needed

### 6. Permissions/Safety locks
- Robot may have safety features preventing commands
- Emergency stop may be engaged
- Solution: Check physical robot state, battery level, etc.

## Quick Test Commands

```bash
# 1. List all active ROS topics
ros2 topic list

# 2. Check for Go2 driver node
ros2 node list | grep -i go2

# 3. See what's publishing to WebRTC topic
ros2 topic info /rt/api/sport/request

# 4. Monitor the topic for activity
ros2 topic echo /rt/api/sport/request

# 5. Check if robot is responding to state queries
ros2 topic echo /go2_states --once

# 6. Try sending stand command directly
ros2 service call /stand std_srvs/srv/Empty
```

## Next Steps

1. **Verify robot driver is running and connected**
2. **Check WebRTC topic has subscribers**
3. **Test direct command bypass (not through DIMOS)**
4. **Verify robot is in correct operational mode**
5. **Check network connectivity to robot**

## Related Files
- Robot Interface: `src/dimos-unitree/dimos/robot/unitree/unitree_ros_control.py`
- Command Queue: `src/dimos-unitree/dimos/robot/ros_command_queue.py`
- WebRTC Topic: Default is `rt/api/sport/request` (line 61 in unitree_ros_control.py)
