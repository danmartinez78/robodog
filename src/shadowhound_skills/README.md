# ShadowHound Vision Skills

Vision capabilities for ShadowHound using DIMOS Qwen VLM integration.

## Overview

This package provides 4 vision skills that enable the robot to understand its visual environment:

1. **vision.snapshot** - Capture and save camera frames
2. **vision.describe_scene** - Get VLM description of what the robot sees
3. **vision.locate_object** - Find specific objects with bounding boxes
4. **vision.detect_objects** - Detect all prominent objects in view

## Architecture

The skills leverage **DIMOS's existing Qwen VLM infrastructure** rather than building from scratch:

- `dimos.models.qwen.video_query.query_single_frame()` - Scene understanding
- `dimos.models.qwen.video_query.get_bbox_from_qwen_frame()` - Object detection

This approach provides:
- ✅ Production-ready VLM integration
- ✅ No redundant infrastructure
- ✅ Lower cost (~$0.001-0.003 per image vs $0.005-0.015 for GPT-4V)
- ✅ Synchronous API for easy testing

## Installation

### Prerequisites

1. **ROS2 Humble** (included in devcontainer)
2. **DIMOS** (already in workspace at `src/dimos-unitree`)
3. **Alibaba Cloud API Key** for Qwen VLM

### Build

```bash
cd /workspaces/shadowhound
colcon build --packages-select shadowhound_skills
source install/setup.bash
```

### Configuration

Set your Alibaba API key (required for VLM skills):

```bash
export ALIBABA_API_KEY='your-api-key-here'
```

Add to `~/.bashrc` for persistence:

```bash
echo 'export ALIBABA_API_KEY="your-key"' >> ~/.bashrc
```

## Usage

### Python API

```python
from shadowhound_skills.vision import SnapshotSkill, DescribeSceneSkill
import numpy as np

# Create test image (or use camera frame)
image = np.zeros((480, 640, 3), dtype=np.uint8)

# Capture snapshot (works without DIMOS)
snapshot = SnapshotSkill()
result = snapshot.execute(image)
print(f"Image saved to: {result.data['image_path']}")

# Describe scene (requires DIMOS + API key)
describe = DescribeSceneSkill()
result = describe.execute(image, query="What do you see?")
if result.success:
    print(f"Description: {result.data['description']}")
else:
    print(f"Error: {result.error}")
```

### Available Skills

#### vision.snapshot
Capture and save current camera frame. **No DIMOS required.**

```python
from shadowhound_skills.vision import SnapshotSkill

skill = SnapshotSkill()
result = skill.execute(image, mission_id="patrol_01", robot_pose=(1.0, 2.0, 0.5))

# Returns:
# {
#   "image_path": "/tmp/shadowhound/images/snapshot_20251007_120000.jpg",
#   "timestamp": "2025-10-07T12:00:00.123456",
#   "size": [640, 480],
#   "format": "JPEG",
#   "mission_id": "patrol_01",
#   "robot_pose": (1.0, 2.0, 0.5)
# }
```

#### vision.describe_scene
Get VLM description of the scene. **Requires DIMOS + ALIBABA_API_KEY.**

```python
from shadowhound_skills.vision import DescribeSceneSkill

skill = DescribeSceneSkill()
result = skill.execute(
    image,
    query="Describe what you see, focusing on potential obstacles",
    save_image=True
)

# Returns:
# {
#   "description": "I see a hallway with...",
#   "query": "Describe what you see...",
#   "image_path": "/tmp/shadowhound/images/describe_20251007_120000.jpg"
# }
```

#### vision.locate_object
Find specific object with bounding box. **Requires DIMOS + ALIBABA_API_KEY.**

```python
from shadowhound_skills.vision import LocateObjectSkill

skill = LocateObjectSkill()
result = skill.execute(image, object_name="coffee cup", save_image=True)

# Returns:
# {
#   "object_name": "coffee cup",
#   "found": True,
#   "bbox": [120, 200, 180, 280],  # [x1, y1, x2, y2]
#   "center": [150, 240],
#   "width": 60,
#   "height": 80,
#   "estimated_size_meters": 0.5,
#   "image_path": "/tmp/shadowhound/images/locate_coffee_cup_20251007.jpg"
# }
```

#### vision.detect_objects
Detect all prominent objects in scene. **Requires DIMOS + ALIBABA_API_KEY.**

```python
from shadowhound_skills.vision import DetectObjectsSkill

skill = DetectObjectsSkill()
result = skill.execute(image, save_image=True)

# Returns:
# {
#   "objects": ["person", "chair", "table", "laptop"],
#   "count": 4,
#   "raw_response": "[\"person\", \"chair\", ...]",
#   "image_path": "/tmp/shadowhound/images/detect_20251007.jpg"
# }
```

## Testing

Run the test suite:

```bash
cd /workspaces/shadowhound
source install/setup.bash
python3 src/shadowhound_skills/test/test_vision.py
```

Expected output:
```
======================================================================
SHADOWHOUND VISION SKILLS TEST SUITE
======================================================================
Available skills: ['vision.snapshot', 'vision.describe_scene', ...]

✅ PASS: Snapshot           (works without DIMOS)
✅ PASS: Describe Scene     (skipped without API key)
✅ PASS: Locate Object      (skipped without API key)
✅ PASS: Detect Objects     (skipped without API key)

🎉 All tests passed!
```

To test with actual VLM calls:
```bash
export ALIBABA_API_KEY="your-key"
python3 src/shadowhound_skills/test/test_vision.py
```

## Architecture Details

### SkillResult Dataclass

All skills return a `SkillResult`:

```python
@dataclass
class SkillResult:
    success: bool
    data: Optional[Dict[str, Any]] = None
    error: Optional[str] = None
    telemetry: Optional[Dict[str, Any]] = None
```

### VisionSkillBase

Base class with shared functionality:

- `require_dimos: bool` - Whether skill needs DIMOS (VLM skills = True, Snapshot = False)
- `_save_image()` - Save PIL image with timestamp
- `_numpy_to_pil()` - Convert numpy array (BGR) to PIL Image (RGB)
- `image_dir` - Default: `/tmp/shadowhound/images/`
- `api_key` - From `ALIBABA_API_KEY` environment variable

### Dependencies

**Always required:**
- rclpy
- pillow
- numpy
- opencv-python

**Required for VLM skills:**
- DIMOS (in workspace at `src/dimos-unitree`)
- ALIBABA_API_KEY environment variable

## Integration with Mission Agent

### Step 1: Wire Camera Feed

```python
# In shadowhound_mission_agent/mission_agent.py
from shadowhound_skills.vision import get_skill

class MissionAgent:
    def __init__(self):
        self.vision_skills = {
            'snapshot': get_skill('vision.snapshot'),
            'describe': get_skill('vision.describe_scene'),
            'locate': get_skill('vision.locate_object'),
            'detect': get_skill('vision.detect_objects'),
        }
    
    def camera_callback(self, msg):
        self.latest_frame = self.bridge.compressed_imgmsg_to_cv2(msg)
        # Now available for vision skills
```

### Step 2: Register with DIMOS MyUnitreeSkills

```python
# In DIMOS integration
from shadowhound_skills.vision import VISION_SKILLS

class MyUnitreeSkills:
    def __init__(self):
        # Register vision skills for function calling
        for skill_name, skill_class in VISION_SKILLS.items():
            self.register_skill(skill_name, skill_class)
```

### Step 3: Mission Execution

```python
# Agent can now call vision skills
result = self.vision_skills['describe'].execute(
    self.latest_frame,
    query="What obstacles do you see ahead?"
)
```

## Cost & Performance

### API Costs (per image)

- **Qwen VLM**: ~$0.001-0.003 per image (Alibaba Cloud)
- **GPT-4V**: ~$0.005-0.015 per image (OpenAI)

**Recommendation**: Use Qwen for routine vision tasks, reserve GPT-4V for complex reasoning.

### Latency

- **Snapshot**: <50ms (local, no network)
- **VLM queries**: 1-3 seconds (network + inference)

### Storage

Images saved to `/tmp/shadowhound/images/` with automatic timestamps:
- `snapshot_YYYYMMDD_HHMMSS_ffffff.jpg`
- `describe_YYYYMMDD_HHMMSS_ffffff.jpg`
- etc.

**Note**: `/tmp/` is cleared on reboot. For production, configure persistent storage.

## Troubleshooting

### "DIMOS vision not available: No module named 'transformers'"

**Issue**: DIMOS dependencies not installed.

**Solution**: VLM skills will fail, but SnapshotSkill works. Install DIMOS dependencies:
```bash
cd /workspaces/shadowhound/src/dimos-unitree
pip install -r requirements.txt
```

### "ALIBABA_API_KEY not set"

**Issue**: VLM skills need API key.

**Solution**: Export the key:
```bash
export ALIBABA_API_KEY='your-key-here'
```

### "Import 'dimos.models.qwen.video_query' could not be resolved"

**Issue**: Pylance can't find DIMOS (expected lint error).

**Solution**: Ignore or add DIMOS to Python path:
```json
// .vscode/settings.json
{
    "python.analysis.extraPaths": ["${workspaceFolder}/src/dimos-unitree"]
}
```

## Roadmap

### Phase 1: Basic Skills (COMPLETE ✅)
- ✅ Snapshot skill
- ✅ Describe scene skill
- ✅ Locate object skill
- ✅ Detect objects skill
- ✅ Test suite

### Phase 2: Integration (NEXT)
- ⏳ Wire mission_agent camera feed
- ⏳ Register with DIMOS MyUnitreeSkills
- ⏳ Test end-to-end vision missions

### Phase 3: Advanced Features (FUTURE)
- ⏸️ OpenAI GPT-4V integration (for complex reasoning)
- ⏸️ Detic/YOLO object detection stream
- ⏸️ Metric3D depth estimation
- ⏸️ Hybrid approach (Qwen for detection, GPT-4V for understanding)

## References

- [DIMOS Vision Capabilities](../../docs/DIMOS_VISION_CAPABILITIES.md)
- [Vision Integration Design](../../docs/VISION_INTEGRATION_DESIGN.md)
- [Camera Architecture](../../docs/CAMERA_ARCHITECTURE.md)
- [ShadowHound Project Context](../../docs/project_context.md)

## License

Apache 2.0
