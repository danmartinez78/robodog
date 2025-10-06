# Web UI Redesign & Mock Image Testing

**Date:** October 6, 2025  
**Status:** ✅ Complete - Ready for testing  
**Branch:** feature/dimos-integration

---

## What We Built

### 1. **Sci-Fi Mission Control UI** 🎨

Complete redesign from "cutesy" purple theme to professional sci-fi aesthetic:

**Visual Design:**
- ✅ Dark background (#0a0e1a) with subtle grid overlay
- ✅ Electric blue (#00d4ff) and cyan (#00ccaa) accents
- ✅ Monospace fonts (Roboto Mono) for technical feel
- ✅ Glowing borders and animated scan lines
- ✅ Angular panels (no rounded corners)
- ✅ Professional "NASA mission control" aesthetic

**UI Layout:**
```
┌─────────────────────────────────────────────────┐
│ SHADOWHOUND // MISSION CONTROL    [STATUS]      │
├────────────────────┬────────────────────────────┤
│  VISUAL FEED       │   MISSION CONTROL          │
│  [Camera Display]  │   [Command Log]            │
│  [Camera Selector] │   [Command Input]          │
│  [Mock Upload]     │   [Execute Button]         │
├────────────────────┴────────────────────────────┤
│  SYSTEM DIAGNOSTICS                             │
│  Status | Battery | Position | Connection       │
└─────────────────────────────────────────────────┘
```

**Removed:**
- ❌ WebRTC skill buttons (sit, stand, wave, etc.)
- ❌ Purple gradient background
- ❌ Rounded, "friendly" design elements

---

## 2. **Mock Image Upload System** 📸

Full implementation for testing vision/VLM features without robot hardware:

### Backend Features (web_interface.py)

**New API Endpoints:**
```python
POST /api/mock/image
  - Upload test images
  - Parameters: image (file), camera (front/left/right)
  - Returns: success, filename, size, camera

GET /api/mock/image/{camera}
  - Retrieve uploaded mock image
  - Returns: base64 encoded image data
```

**Storage:**
- Images saved to: `~/.shadowhound/mock_images/`
- In-memory cache: `self.mock_images` dict
- Accessible to vision skills via `get_mock_image(camera)`

**Methods for Vision Skills:**
```python
web.get_mock_image(camera="front")  → bytes or None
web.has_mock_image(camera="front")  → bool
```

### Frontend Features (JavaScript)

**Camera Feed Display:**
- Live preview of uploaded images
- Camera selector (Front/Left/Right)
- "NO SIGNAL" placeholder when empty

**Upload Interface:**
- File upload button with drag-and-drop styling
- Instant preview in camera feed
- Upload to backend with camera selection
- Real-time status logging

**WebSocket Updates:**
- Live connection status
- Command execution logging
- Mock image upload notifications
- System diagnostics

---

## How to Test

### 1. Start Web Interface

**Option A: Standalone (what we just did)**
```bash
cd /home/daniel/shadowhound
python3 src/shadowhound_mission_agent/shadowhound_mission_agent/web_interface.py
```

**Option B: With Mission Agent (when robot is available)**
```bash
ros2 launch shadowhound_bringup shadowhound_bringup.launch.py
```

### 2. Open Browser
Navigate to: http://localhost:8080

You should see:
- Dark sci-fi themed interface
- "SHADOWHOUND // AUTONOMOUS MISSION CONTROL" header
- Camera feed panel (top-left)
- Mission control panel (right side)
- System diagnostics panel (bottom-left)

### 3. Test Mock Image Upload

**Step-by-step:**
1. Click "UPLOAD TEST IMAGE" button in camera feed panel
2. Select any image from your laptop
3. Image should appear in camera feed immediately
4. Check mission log for "MOCK IMAGE UPLOADED" confirmation
5. Try switching cameras (Front/Left/Right)
6. Upload different images for each camera

**What to use as test images:**
- Photos of rooms in your house
- Stock images from the internet
- Screenshots
- Any JPG/PNG file

**Example workflow:**
```
1. Upload kitchen photo → Front camera
2. Upload living room photo → Left camera
3. Upload hallway photo → Right camera
4. Switch between cameras to see different views
```

### 4. Test Command Interface

**Try these commands:**
```
"navigate to kitchen"
"what do you see?"
"rotate 90 degrees"
"take a photo"
```

The commands will execute through the mission agent (when connected).

### 5. Verify Mock Image Access

**From Python (later when we build vision skills):**
```python
from shadowhound_mission_agent.web_interface import WebInterface

# Get mock image
image_data = web_interface.get_mock_image("front")
if image_data:
    print(f"Retrieved {len(image_data)} bytes")
    # Use with VLM API, save to disk, etc.
```

---

## Next Steps: Vision Skills Integration

Now that we have mock image capability, we can build vision skills:

### Phase 1: Vision Skill (vision.py)
```python
@register_skill("vision.snapshot")
class SnapshotSkill(Skill):
    """Capture image from camera (mock or real)."""
    
    def execute(self, camera: str = "front") -> SkillResult:
        # Check if mock mode
        if web_interface.has_mock_image(camera):
            image_data = web_interface.get_mock_image(camera)
            # Save to disk, return path
        else:
            # Subscribe to ROS camera topic
            pass
```

### Phase 2: VLM Integration
```python
@register_skill("vision.describe")
class DescribeSkill(Skill):
    """Use GPT-4 Vision to describe image."""
    
    def execute(self) -> SkillResult:
        # Get latest snapshot
        image_path = get_latest_snapshot()
        
        # Send to OpenAI GPT-4 Vision
        response = openai.chat.completions.create(
            model="gpt-4-vision-preview",
            messages=[{
                "role": "user",
                "content": [
                    {"type": "text", "text": "Describe what you see"},
                    {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"}}
                ]
            }]
        )
        
        return SkillResult(
            success=True,
            data={"description": response.choices[0].message.content}
        )
```

### Phase 3: Test Full Flow
```
1. Upload test image via web UI
2. Send command: "what do you see?"
3. Mission agent:
   - Calls vision.snapshot skill
   - Retrieves mock image
   - Calls vision.describe skill
   - Sends image to GPT-4 Vision
   - Returns description
4. Result appears in mission log
```

---

## Technical Details

### Files Modified
```
src/shadowhound_mission_agent/shadowhound_mission_agent/web_interface.py
  - Added imports: os, base64, Path, File, UploadFile, Form
  - Added mock_images dict and mock_image_dir
  - Added POST /api/mock/image endpoint
  - Added GET /api/mock/image/{camera} endpoint
  - Added get_mock_image() method
  - Added has_mock_image() method
  - Complete HTML/CSS redesign (500+ lines)
  - Updated JavaScript for image upload
```

### Mock Image Storage
```
~/.shadowhound/mock_images/
├── mock_front_kitchen.jpg
├── mock_left_livingroom.jpg
└── mock_right_hallway.png
```

### API Examples

**Upload Image (curl):**
```bash
curl -X POST http://localhost:8080/api/mock/image \
  -F "image=@/path/to/test.jpg" \
  -F "camera=front"
```

**Get Image (curl):**
```bash
curl http://localhost:8080/api/mock/image/front
```

---

## Benefits of This Approach

✅ **No Robot Required**: Test vision/VLM integration from your laptop
✅ **Instant Feedback**: Upload image and see it immediately
✅ **Multiple Cameras**: Test all three camera angles
✅ **Realistic Testing**: Use real-world images similar to robot's view
✅ **VLM Development**: Build and test AI image understanding offline
✅ **Professional UI**: Demo-ready interface for presentations

---

## Known Limitations

⚠️ **Current State:**
- Web UI redesigned ✅
- Mock image upload implemented ✅
- Vision skills not yet implemented 🔨
- VLM integration not yet implemented 🔨

⚠️ **Future Work:**
- Build vision.py skills module
- Integrate with OpenAI GPT-4 Vision API
- Add real camera feed from robot (when connected)
- Add BEV lidar visualization
- Add battery/position telemetry from robot

---

## Testing Checklist

Before declaring success, verify:

- [ ] Web UI loads at http://localhost:8080
- [ ] Dark sci-fi theme displays correctly
- [ ] WebSocket connection shows "CONNECTED"
- [ ] Can upload image via "UPLOAD TEST IMAGE" button
- [ ] Image appears in camera feed
- [ ] Mission log shows "MOCK IMAGE UPLOADED" message
- [ ] Can switch between Front/Left/Right cameras
- [ ] Different images can be uploaded per camera
- [ ] Command input accepts text
- [ ] Diagnostics panel displays status
- [ ] No console errors in browser

---

## Troubleshooting

### Web UI doesn't load
```bash
# Check if port is in use
lsof -i :8080

# Kill existing process
pkill -f web_interface.py

# Restart
python3 src/shadowhound_mission_agent/shadowhound_mission_agent/web_interface.py
```

### WebSocket won't connect
- Check firewall settings
- Verify port 8080 is accessible
- Check browser console for errors

### Image upload fails
```bash
# Verify directory exists
ls -la ~/.shadowhound/mock_images/

# Check permissions
chmod 755 ~/.shadowhound/mock_images/
```

### UI looks broken
- Clear browser cache (Ctrl+Shift+R)
- Check browser console for errors
- Verify HTML loaded correctly (view source)

---

## Summary

We've successfully built a **professional sci-fi themed web UI** with **mock image upload capability** that enables you to:

1. ✅ Test and develop vision/VLM features without robot hardware
2. ✅ Upload test images from your laptop
3. ✅ See realistic camera feed simulation
4. ✅ Have a demo-ready interface for presentations

**Next milestone:** Build vision skills that use these mock images with GPT-4 Vision API.

**Time saved:** Days of debugging robot hardware, can now work on couch! 🛋️
