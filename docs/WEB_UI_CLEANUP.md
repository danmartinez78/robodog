# Web UI Cleanup - October 6, 2025

## Issues Identified
1. **Mixed UI Elements**: Merge reintroduced old "cutesy" UI elements (emojis, quick command buttons)
2. **Template Not Being Installed**: `setup.py` missing `package_data` configuration
3. **Camera Feed Not Working**: Still showing "NO FEED AVAILABLE"

## Changes Made

### 1. UI Cleanup (Commit: 386ef2b)
**Removed:**
- 🐕 Dog emoji from page title and header
- All emoji icons from section headers (📹, 📊, 💻, 📡, 🎮)
- Quick command buttons: STAND, SIT, WAVE, DANCE, STRETCH, BALANCE
- Unused `.quick-commands` CSS class

**Updated:**
- Header: "SHADOWHOUND" (clean, no emoji)
- Section titles: Professional text only
- Command placeholder: "Patrol the perimeter and report any anomalies" (mission-focused)
- Button text: "EXECUTE" (no emoji)

### 2. Package Installation Fix
**Fixed `setup.py`:**
```python
package_data={
    package_name: ["dashboard_template.html"],
},
include_package_data=True,
```

This ensures the HTML template is installed with the package and not falling back to the old embedded HTML in `web_interface.py`.

## Testing Instructions

### 1. Verify UI Cleanup
```bash
cd /home/daniel/shadowhound
source install/setup.bash

# Launch the system
ros2 launch shadowhound_mission_agent mission_agent.launch.py

# Open browser
firefox http://localhost:8080
```

**Expected Results:**
- ✅ Clean header: "SHADOWHOUND" (no dog emoji)
- ✅ Clean section titles (no emojis)
- ✅ Only "Natural Language Command" input field
- ✅ Single "EXECUTE" button (no quick commands)

### 2. Diagnose Camera Feed
```bash
# Run diagnostic script
./scripts/diagnose_camera.sh
```

**What to check:**
1. Is `/camera/compressed` topic publishing?
2. Is mission_agent running and subscribed?
3. What is the QoS profile of the camera topic?
4. Is the web interface responding?

### 3. Test Camera Streaming
```bash
# In one terminal - check if topic is active
ros2 topic hz /camera/compressed

# In another terminal - monitor mission agent logs
ros2 launch shadowhound_mission_agent mission_agent.launch.py
# Look for: "📷 Subscribed to camera feed: /camera/compressed"
```

**Browser Developer Console (F12):**
- Check for WebSocket messages starting with "CAMERA_FRAME:"
- Check for any JavaScript errors
- Verify WebSocket status shows "CONNECTED"

## Known Issues

### Camera Feed Not Displaying
**Possible causes:**
1. Camera topic not publishing (robot not connected)
2. QoS mismatch (should be BEST_EFFORT)
3. WebSocket not receiving frames
4. mission_agent not calling `web.broadcast_sync()`

**Next Steps:**
1. Run diagnostic script to identify root cause
2. Check if robot is connected and camera is publishing
3. Add more logging to `_camera_callback` to see if frames are being received
4. Test with mock camera publisher if robot unavailable

## Files Modified
- `src/shadowhound_mission_agent/shadowhound_mission_agent/dashboard_template.html`
- `src/shadowhound_mission_agent/setup.py`

## Files Created
- `scripts/diagnose_camera.sh` - Diagnostic tool for camera troubleshooting
- `docs/WEB_UI_CLEANUP.md` - This document

## Related Documentation
- `docs/CAMERA_FEED_INTEGRATION.md` - Camera streaming technical details
- `docs/MERGE_RESOLUTION_2025-10-06.md` - Details of feature branch merge
