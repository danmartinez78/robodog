# Web UI Cache Clearing Guide

If you're seeing an old version of the web UI, it's likely a browser cache issue.

## Quick Fix

### Option 1: Hard Refresh (Fastest)
**Windows/Linux**: `Ctrl + Shift + R` or `Ctrl + F5`
**Mac**: `Cmd + Shift + R`

### Option 2: Clear Cache for Site
**Chrome/Edge**:
1. Press `F12` (open DevTools)
2. Right-click the refresh button
3. Select "Empty Cache and Hard Reload"

**Firefox**:
1. Press `Ctrl + Shift + Delete`
2. Select "Cached Web Content"
3. Click "Clear Now"

### Option 3: Incognito/Private Window
Open http://localhost:8080 in an incognito/private window to bypass cache entirely.

### Option 4: Clear All Browser Cache
**Chrome/Edge**: Settings → Privacy → Clear browsing data → Cached images and files
**Firefox**: Settings → Privacy & Security → Clear Data → Cached Web Content

## Verify the Fix

After clearing cache, you should see:
- ⚡ Performance Metrics panel (top right)
- 📊 Diagnostics panel (bottom left)
- 💻 Terminal with clean output (no duplicate timing)
- 📹 Camera feed (top left)

## Still Having Issues?

If cache clearing doesn't work:

1. **Check the server is running the latest code**:
   ```bash
   # On the robot/dev machine
   cd /workspaces/shadowhound
   colcon build --packages-select shadowhound_mission_agent --symlink-install
   source install/setup.bash
   
   # Restart the node
   ros2 launch shadowhound_bringup shadowhound.launch.py
   ```

2. **Verify you're on the right branch**:
   ```bash
   git branch
   # Should show: * feature/dimos-integration
   
   git log --oneline -3
   # Should show recent commits including performance metrics
   ```

3. **Check build directory isn't stale**:
   ```bash
   # Nuclear option: clean rebuild
   rm -rf build/ install/ log/
   colcon build --symlink-install
   source install/setup.bash
   ```

4. **Verify the file on disk**:
   ```bash
   grep -n "PERFORMANCE METRICS" src/shadowhound_mission_agent/shadowhound_mission_agent/dashboard_template.html
   # Should show the performance panel exists
   ```

## Expected UI Layout

```
┌─────────────────────────────────┬─────────────────────────────────┐
│  📹 CAMERA FEED                 │  ⚡ PERFORMANCE METRICS         │
│                                 │  Agent: 1.23s                   │
│  [Camera image here]            │  Overhead: 0.045s               │
│                                 │  Total: 1.28s                   │
│                                 │  Commands: 42                   │
└─────────────────────────────────┴─────────────────────────────────┘
┌─────────────────────────────────┬─────────────────────────────────┐
│  📊 DIAGNOSTICS                 │  💻 TERMINAL                    │
│  Robot Mode: STANDING           │  > command                      │
│  Topics: 12                     │  ✅ response                    │
│                                 │  ⏱️ timing                      │
└─────────────────────────────────┴─────────────────────────────────┘
```

If you're seeing the OLD layout (without Performance Metrics panel), it's definitely a cache issue!
