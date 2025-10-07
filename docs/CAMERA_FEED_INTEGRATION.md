# Camera Feed Integration - Testing Guide

## What Was Changed

Added live camera feed to the web interface so you can see what the robot sees directly in the browser.

### Changes Made

1. **mission_agent.py**:
   - Added subscription to `/camera/compressed` topic
   - Uses BEST_EFFORT QoS to match camera publisher
   - Forwards camera frames via WebSocket to web interface
   - Added `_camera_callback()` method to handle incoming frames

2. **web_interface.py**:
   - Updated WebSocket message handler to detect camera frames
   - Camera frames sent as `CAMERA_FRAME:<base64_data>`
   - JavaScript automatically displays frames in camera panel

## How to Test

### 1. Restart the Mission Agent

If you have the system running, restart it to pick up the changes:

```bash
# Kill existing agent
pkill -f mission_agent

# Or use the full restart
./start.sh --dev --skip-update
```

### 2. Check the Logs

You should see this line in the startup logs:

```
[mission_agent-1] [INFO] [timestamp] [mission_agent]: 📷 Subscribed to camera feed: /camera/compressed
```

### 3. Open the Web Interface

Navigate to: http://localhost:8080

You should now see:
- Live camera feed in the camera panel (top-right)
- Updates in real-time as the robot moves
- No lag or delay (streaming directly via WebSocket)

### 4. Verify Camera Topic

Check that the camera topic is publishing:

```bash
# List camera topics
ros2 topic list | grep camera

# Check publication rate
ros2 topic hz /camera/compressed

# Echo a few frames
ros2 topic echo /camera/compressed --once
```

## Troubleshooting

### Camera Feed Not Showing

**Check 1**: Is the camera topic publishing?
```bash
ros2 topic list | grep camera
```

**Check 2**: Is the mission agent subscribed?
```bash
ros2 node info /mission_agent | grep -A 10 Subscriptions
```
Should show `/camera/compressed`

**Check 3**: Check browser console (F12)
- Look for WebSocket messages
- Should see "WebSocket connected"
- Camera frames should NOT appear in console (filtered out to reduce spam)

**Check 4**: Verify QoS compatibility
```bash
# Check camera topic QoS
ros2 topic info /camera/compressed --verbose
```
Should show BEST_EFFORT reliability

### Camera Feed is Laggy

This shouldn't happen with WebSocket streaming, but if it does:

1. **Check network**: Ensure good connection between laptop and robot
2. **Check CPU**: High CPU usage can cause frame drops
3. **Check bandwidth**: Camera resolution might be too high

### Error Messages

**"Failed to subscribe to camera"**
- sensor_msgs package might not be installed
- Run: `sudo apt install ros-humble-sensor-msgs`

**"Camera callback error"**
- Check the error count in logs
- Usually indicates format issues with camera data

## Technical Details

### Camera Stream Flow

```
Robot Camera
    ↓
/camera/compressed (CompressedImage, BEST_EFFORT QoS)
    ↓
mission_agent._camera_callback()
    ↓
base64 encode
    ↓
WebSocket: "CAMERA_FRAME:<base64>"
    ↓
web_interface JavaScript
    ↓
Display in <img> tag
```

### Performance

- **Latency**: ~50-100ms (WebSocket + encoding)
- **Frame Rate**: Matches camera publisher (typically 10-30 fps)
- **Bandwidth**: ~50-200 KB/frame (depends on compression)

### QoS Settings

Using BEST_EFFORT QoS because:
- Camera feed is real-time data (old frames are useless)
- Missing frames are acceptable
- Reduces latency
- Matches typical camera publisher settings

## Next Steps

If camera feed is working:

1. ✅ Test with robot moving around
2. ✅ Verify frame rate is acceptable
3. ✅ Check web interface responsiveness
4. ✅ Test mission execution while viewing feed

If you want to improve it:

- Add frame rate indicator
- Add "freeze frame" button
- Add full-screen camera view
- Add multiple camera support (front/rear/side)
- Add recording capability

## Commit

Changes committed as:
```
feat: Add live camera feed to web interface

- Subscribe to /camera/compressed topic in mission_agent
- Forward camera frames via WebSocket to web interface
- Update JavaScript to display camera frames in real-time
- Use BEST_EFFORT QoS to match camera publisher
- Add error handling to avoid log spam
```

Branch: `feature/agent-refactor`

---

**Quick Test**: Open http://localhost:8080 and you should see the live camera feed in the top-right panel!
