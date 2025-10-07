# Laptop Diagnostic Commands

Run these commands **on your laptop** to check what's actually deployed:

## 1. Check which branch is running
```bash
cd ~/shadowhound  # or wherever your workspace is
git branch
git log --oneline -5
```

**Expected**: Should show `feature/dimos-integration` branch with recent commits including:
- `a3d4474` docs: Document timing display fix
- `0c106cd` fix: Combine timing info with response
- `8d6be4b` docs: Add push summary

## 2. Check the dashboard file
```bash
grep -n "🐕" src/shadowhound_mission_agent/shadowhound_mission_agent/dashboard_template.html
```

**If found**: You have the OLD dashboard (🐕 SHADOWHOUND title)
**If NOT found**: Dashboard was updated but not deployed

Then check for performance metrics:
```bash
grep -n "PERFORMANCE METRICS" src/shadowhound_mission_agent/shadowhound_mission_agent/dashboard_template.html
```

**Expected**: Should find 5+ matches if performance panel exists

## 3. Check what's actually installed
```bash
ls -la install/shadowhound_mission_agent/lib/python3.10/site-packages/shadowhound_mission_agent/
```

Look for `dashboard_template.html` timestamp - when was it last updated?

## 4. Pull latest changes
```bash
git pull origin feature/dimos-integration
```

**This will sync your laptop with the devcontainer commits**

## 5. Rebuild on laptop
```bash
colcon build --packages-select shadowhound_mission_agent --symlink-install
source install/setup.bash
```

## 6. Restart the node
```bash
# Kill existing node
pkill -f mission_agent

# Restart
ros2 launch shadowhound_bringup shadowhound.launch.py
```

## 7. Clear browser cache AGAIN
After restarting with new code:
- **Hard refresh**: `Ctrl + Shift + R` (Windows/Linux) or `Cmd + Shift + R` (Mac)
- Or use incognito window

## Expected Result

After pulling, rebuilding, and cache clearing, you should see:

**Page Title**: `🐕 ShadowHound Mission Control` (no dog emoji in HTML, just text)
**Layout**:
```
┌─────────────────────────┬─────────────────────────┐
│ 📹 CAMERA FEED         │ ⚡ PERFORMANCE METRICS  │
├─────────────────────────┼─────────────────────────┤
│ 📊 DIAGNOSTICS         │ 💻 TERMINAL             │
└─────────────────────────┴─────────────────────────┘
```

**NO quick command buttons** at the bottom - those were removed in favor of clean command input.

## Still Seeing Old UI?

If you still see:
- 🐕 emoji in browser tab title
- Quick command buttons (STAND, SIT, WAVE, etc.)
- NO Performance Metrics panel

Then either:
1. Git pull didn't get latest commits (check `git log`)
2. Build didn't complete (`colcon build` had errors?)
3. Node is still running old code (restart it)
4. Browser is REALLY holding onto cache (try different browser)

## Quick Test

Run this to see exact title in your dashboard:
```bash
head -5 src/shadowhound_mission_agent/shadowhound_mission_agent/dashboard_template.html
```

Should show:
```html
<!DOCTYPE html>
<html>

<head>
    <title>🐕 ShadowHound Mission Control</title>
```

If you see emoji as actual emoji character (🐕) instead of HTML entity, that's fine.
But if the whole layout is different, you need to pull and rebuild!
