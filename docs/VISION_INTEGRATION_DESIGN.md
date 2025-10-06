# Vision Integration Design

## Overview

This document outlines the architecture for integrating vision capabilities into ShadowHound, enabling the agent to process camera images and make vision-based decisions. The design leverages our existing agent architecture and mock image upload system.

## Architecture Options

### Option 1: Vision Skills (Recommended)

Add vision capabilities as **skills** that agents can call, keeping the agent architecture unchanged.

```
┌─────────────────────────────────────────────────────────────┐
│                    Agent (OpenAI/Planning)                  │
│  "Go to the red object on the table"                        │
└─────────────────────┬───────────────────────────────────────┘
                      │
                      ▼
        ┌─────────────────────────────────────────┐
        │         Skills Registry                 │
        │  - nav.goto(x, y, yaw)                  │
        │  - nav.rotate(yaw)                      │
        │  - perception.snapshot()         ◄──────┼── NEW
        │  - vision.detect_objects()       ◄──────┼── NEW
        │  - vision.locate_object(name)    ◄──────┼── NEW
        │  - vision.describe_scene()       ◄──────┼── NEW
        └─────────────────────────────────────────┘
                      │
                      ▼
        ┌─────────────────────────────────────────┐
        │      Vision Skill Implementation        │
        │  - Get image from robot/mock            │
        │  - Call GPT-4 Vision API                │
        │  - Return structured results            │
        └─────────────────────────────────────────┘
```

**Pros:**
- ✅ No changes to agent architecture
- ✅ Works with any agent type (OpenAI, Planning, Custom)
- ✅ Skills are reusable across different missions
- ✅ Clear separation: agent = planning, skills = execution
- ✅ Easy to test skills independently
- ✅ Can mix vision and non-vision skills in same mission

**Cons:**
- ❌ Agent doesn't directly see images (must call skills)
- ❌ Two-step process: agent plans → skill executes vision

### Option 2: Vision-Enabled Agent

Create a specialized **VisionAgent** that has direct access to camera and uses GPT-4 Vision.

```
┌─────────────────────────────────────────────────────────────┐
│                AgentFactory.create()                        │
└─────────────────────┬───────────────────────────────────────┘
                      │
          ┌───────────┼───────────┬──────────────┐
          ▼           ▼           ▼              ▼
    ┌──────────┐ ┌─────────┐ ┌──────────┐ ┌──────────────┐
    │ OpenAI   │ │Planning │ │ Custom   │ │ VisionAgent  │◄── NEW
    │ Agent    │ │ Agent   │ │ Agent    │ │ (GPT-4V)     │
    └──────────┘ └─────────┘ └──────────┘ └──────────────┘
                                                  │
                                                  ▼
                                          ┌──────────────┐
                                          │ Camera/Mock  │
                                          │ Image Source │
                                          └──────────────┘
```

**Pros:**
- ✅ Agent directly processes images
- ✅ Single-step reasoning with vision context
- ✅ Better for complex vision-based missions
- ✅ More natural for tasks like "find and go to X"

**Cons:**
- ❌ New agent type to maintain
- ❌ Duplicate logic between OpenAIAgent and VisionAgent
- ❌ Less flexible - vision always enabled
- ❌ More complex to test

### Option 3: Hybrid - Vision Skills + Vision Context

Combine both approaches: vision skills for execution + optional image context for planning.

```
┌─────────────────────────────────────────────────────────────┐
│              VisionContextAgent                             │
│  Inherits from OpenAIAgent                                  │
│  + Can attach images to planning context                    │
└─────────────────────┬───────────────────────────────────────┘
                      │
                      ├─► Plan with image context (GPT-4V)
                      │
                      └─► Execute via vision skills
```

**Pros:**
- ✅ Best of both worlds
- ✅ Agent can "see" during planning
- ✅ Skills handle execution details
- ✅ Flexible - can enable/disable vision context

**Cons:**
- ❌ Most complex implementation
- ❌ Requires careful prompt engineering
- ❌ Higher token costs (images in every planning call)

## Recommended Approach: Vision Skills (Option 1)

### Why Vision Skills?

1. **Simplest to implement** - No agent architecture changes
2. **Testable** - Works with our existing mock image system
3. **Flexible** - Can be used by any agent type
4. **Cost-effective** - Only call GPT-4V when skill is executed
5. **Composable** - Mix vision and non-vision skills naturally

## Implementation Plan

### Phase 1: Core Vision Skills

Create a new skills module with basic vision capabilities:

```python
# shadowhound_skills/vision.py

from shadowhound_skills.skill_base import Skill, SkillResult
from shadowhound_skills.skill_registry import register_skill
import base64
import requests
import os

@register_skill("perception.snapshot")
class SnapshotSkill(Skill):
    """Capture an image from the camera (or mock).
    
    Returns:
        SkillResult with image data and metadata
    """
    
    def execute(self, camera: str = "front", **kwargs) -> SkillResult:
        # Get image from robot or mock
        image_data = self._get_image(camera)
        
        if image_data is None:
            return SkillResult(
                success=False,
                error=f"No image available from {camera} camera"
            )
        
        # Save to temporary location or return as base64
        image_b64 = base64.b64encode(image_data).decode('utf-8')
        
        return SkillResult(
            success=True,
            data={
                "camera": camera,
                "image_base64": image_b64,
                "size_bytes": len(image_data)
            },
            telemetry={"image_size": len(image_data)}
        )
    
    def _get_image(self, camera: str) -> bytes:
        """Get image from robot interface or mock."""
        # Check for mock image first (for testing)
        if hasattr(self.robot_interface, 'get_mock_image'):
            mock_img = self.robot_interface.get_mock_image(camera)
            if mock_img:
                return mock_img
        
        # Fall back to real camera
        return self.robot_interface.get_camera_image(camera)


@register_skill("vision.describe_scene")
class DescribeSceneSkill(Skill):
    """Describe what's in the current camera view using GPT-4 Vision.
    
    Args:
        camera: Which camera to use (default: 'front')
        detail: Level of detail ('low', 'high', 'auto')
    
    Returns:
        SkillResult with text description of the scene
    """
    
    def execute(self, camera: str = "front", detail: str = "auto", **kwargs) -> SkillResult:
        # Get image
        image_data = self._get_image(camera)
        if image_data is None:
            return SkillResult(success=False, error="No image available")
        
        # Call GPT-4 Vision
        try:
            description = self._call_gpt4_vision(
                image_data,
                prompt="Describe what you see in this image. Be specific about objects, their locations, and any relevant details.",
                detail=detail
            )
            
            return SkillResult(
                success=True,
                data={
                    "description": description,
                    "camera": camera
                },
                telemetry={
                    "tokens_used": len(description.split()),
                    "detail_level": detail
                }
            )
        except Exception as e:
            return SkillResult(
                success=False,
                error=f"Vision API error: {str(e)}"
            )
    
    def _call_gpt4_vision(self, image_data: bytes, prompt: str, detail: str) -> str:
        """Call OpenAI GPT-4 Vision API."""
        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            raise ValueError("OPENAI_API_KEY not set")
        
        image_b64 = base64.b64encode(image_data).decode('utf-8')
        
        response = requests.post(
            "https://api.openai.com/v1/chat/completions",
            headers={
                "Authorization": f"Bearer {api_key}",
                "Content-Type": "application/json"
            },
            json={
                "model": "gpt-4-vision-preview",
                "messages": [
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": prompt},
                            {
                                "type": "image_url",
                                "image_url": {
                                    "url": f"data:image/jpeg;base64,{image_b64}",
                                    "detail": detail
                                }
                            }
                        ]
                    }
                ],
                "max_tokens": 500
            }
        )
        
        response.raise_for_status()
        return response.json()["choices"][0]["message"]["content"]


@register_skill("vision.detect_objects")
class DetectObjectsSkill(Skill):
    """Detect and locate objects in the camera view.
    
    Args:
        camera: Which camera to use
        object_types: List of object types to detect (optional)
    
    Returns:
        SkillResult with list of detected objects and their locations
    """
    
    def execute(self, camera: str = "front", object_types: list = None, **kwargs) -> SkillResult:
        image_data = self._get_image(camera)
        if image_data is None:
            return SkillResult(success=False, error="No image available")
        
        # Build prompt based on object types
        if object_types:
            prompt = f"List all {', '.join(object_types)} visible in this image. For each object, describe its location (left/center/right, near/far)."
        else:
            prompt = "List all objects visible in this image with their approximate locations (left/center/right, near/far)."
        
        try:
            detection_text = self._call_gpt4_vision(image_data, prompt, detail="high")
            
            # Parse the response into structured data
            objects = self._parse_object_list(detection_text)
            
            return SkillResult(
                success=True,
                data={
                    "objects": objects,
                    "count": len(objects),
                    "raw_response": detection_text
                }
            )
        except Exception as e:
            return SkillResult(success=False, error=str(e))
    
    def _parse_object_list(self, text: str) -> list:
        """Parse GPT-4V response into structured object list."""
        # Simple parsing - split by lines, extract object info
        objects = []
        for line in text.split('\n'):
            line = line.strip()
            if line and not line.startswith('#'):
                objects.append({
                    "description": line,
                    "confidence": "high"  # GPT-4V doesn't give confidence scores
                })
        return objects


@register_skill("vision.locate_object")
class LocateObjectSkill(Skill):
    """Find a specific object and determine how to reach it.
    
    Args:
        object_name: Name of object to find (e.g., "red cup", "person")
        camera: Which camera to use
    
    Returns:
        SkillResult with object location and navigation suggestion
    """
    
    def execute(self, object_name: str, camera: str = "front", **kwargs) -> SkillResult:
        image_data = self._get_image(camera)
        if image_data is None:
            return SkillResult(success=False, error="No image available")
        
        prompt = f"""
        Look for a {object_name} in this image. If you find it:
        1. Describe its location (left/center/right, near/far)
        2. Estimate the angle to turn (degrees left/right from center)
        3. Estimate the distance (in robot body lengths, roughly)
        
        Respond in this format:
        FOUND: yes/no
        LOCATION: <description>
        ANGLE: <degrees, negative for left, positive for right>
        DISTANCE: <approximate distance>
        """
        
        try:
            response = self._call_gpt4_vision(image_data, prompt, detail="high")
            
            # Parse response
            location_data = self._parse_location_response(response)
            
            if location_data.get("found"):
                return SkillResult(
                    success=True,
                    data=location_data,
                    message=f"Found {object_name}: {location_data['location']}"
                )
            else:
                return SkillResult(
                    success=False,
                    error=f"{object_name} not found in camera view",
                    data=location_data
                )
        except Exception as e:
            return SkillResult(success=False, error=str(e))
    
    def _parse_location_response(self, text: str) -> dict:
        """Parse GPT-4V location response."""
        result = {
            "found": False,
            "location": "",
            "angle": 0.0,
            "distance": 0.0,
            "raw_response": text
        }
        
        for line in text.split('\n'):
            line = line.strip()
            if line.startswith("FOUND:"):
                result["found"] = "yes" in line.lower()
            elif line.startswith("LOCATION:"):
                result["location"] = line.split(":", 1)[1].strip()
            elif line.startswith("ANGLE:"):
                try:
                    result["angle"] = float(line.split(":", 1)[1].strip().split()[0])
                except:
                    pass
            elif line.startswith("DISTANCE:"):
                try:
                    result["distance"] = float(line.split(":", 1)[1].strip().split()[0])
                except:
                    pass
        
        return result
```

### Phase 2: Integration with Robot Interface

Update the robot interface to provide vision capabilities:

```python
# shadowhound_robot/robot_interface.py

class RobotInterface:
    """Bridge to robot hardware and mock data."""
    
    def __init__(self, web_interface=None):
        self.web_interface = web_interface
        # ... existing init
    
    def get_camera_image(self, camera: str = "front") -> Optional[bytes]:
        """Get image from robot camera or mock.
        
        Priority:
        1. Check for mock image (for testing)
        2. Fall back to real camera feed
        """
        # Check mock first (for testing without robot)
        if self.web_interface and hasattr(self.web_interface, 'get_mock_image'):
            mock_img = self.web_interface.get_mock_image(camera)
            if mock_img:
                return mock_img
        
        # Get from real camera
        return self._get_real_camera_image(camera)
    
    def _get_real_camera_image(self, camera: str) -> Optional[bytes]:
        """Get image from actual robot camera."""
        # Subscribe to /camera/image_raw or similar
        # Return latest frame as JPEG bytes
        pass
```

### Phase 3: Agent Integration

Agents can now call vision skills naturally:

```python
# Example agent conversation with GPT-4:

# User: "Go to the red cup on the table"

# Agent Plans:
# 1. vision.describe_scene() - Get overview
# 2. vision.locate_object(object_name="red cup") - Find target
# 3. nav.rotate(yaw=<angle from vision>) - Face object
# 4. nav.goto(x=<calculated>, y=<calculated>) - Move to it
# 5. vision.describe_scene() - Verify we're there

# Each skill returns structured results that the agent can use
```

## Testing Strategy

### Level 1: Unit Tests (No Robot)

```python
def test_vision_skill_with_mock_image():
    """Test vision skill using mock image."""
    # Load test image
    with open("test_images/kitchen.jpg", "rb") as f:
        test_image = f.read()
    
    # Create mock robot interface
    mock_robot = Mock()
    mock_robot.get_camera_image.return_value = test_image
    
    # Test skill
    skill = DescribeSceneSkill(robot_interface=mock_robot)
    result = skill.execute(camera="front")
    
    assert result.success
    assert "description" in result.data
    assert len(result.data["description"]) > 0
```

### Level 2: Web UI Testing (Manual)

1. Upload test image via web interface
2. Send mission command: "Describe what you see"
3. Agent calls `vision.describe_scene()`
4. Skill retrieves mock image from web interface
5. Returns description to agent
6. Result displayed in web UI

### Level 3: Integration Testing (With Robot)

1. Robot's camera provides real feed
2. Agent executes vision-based missions
3. Skills access real camera data
4. Verify behavior matches expectations

## Example Missions

### Mission 1: "Describe your surroundings"

```
Agent → vision.describe_scene(camera="front")
Skill → Returns: "I see a kitchen table with a red cup, a laptop, and a bowl of fruit..."
Agent → report.say(text="I can see a kitchen table...")
```

### Mission 2: "Go to the red cup"

```
Agent → vision.locate_object(object_name="red cup")
Skill → Returns: {found: true, angle: 15, distance: 2.0, location: "right side of table"}
Agent → nav.rotate(yaw=0.26)  # 15 degrees
Agent → nav.goto(x=2.0, y=0.5)
Agent → vision.describe_scene()
Skill → Returns: "The red cup is now directly in front of me..."
Agent → report.say(text="I have reached the red cup")
```

### Mission 3: "Find the person and wave"

```
Agent → vision.detect_objects(object_types=["person"])
Skill → Returns: {objects: [{description: "person standing on left"...}]}
Agent → vision.locate_object(object_name="person")
Skill → Returns: {found: true, angle: -30, distance: 3.0}
Agent → nav.rotate(yaw=-0.52)  # -30 degrees
Agent → gestures.wave()
Agent → report.say(text="Hello!")
```

## Cost Considerations

GPT-4 Vision API costs:
- **Low detail**: ~$0.00085 per image (85 tokens)
- **High detail**: ~$0.00765 per image (765 tokens)
- **Auto**: API decides based on image size

**Optimization strategies:**
1. Cache descriptions for recently seen scenes
2. Use low detail for overview, high detail only when needed
3. Batch vision operations when possible
4. Fall back to non-vision methods when appropriate

## Future Enhancements

### 1. Vision Context Agent (Option 3)

Once vision skills are stable, add optional vision context:

```python
class VisionContextAgent(OpenAIAgent):
    """Agent that includes camera view in planning context."""
    
    def execute_mission(self, command: str) -> MissionResult:
        # Get current camera image
        image = self.robot_interface.get_camera_image()
        
        # Add image to planning context
        context_with_vision = self._add_vision_context(command, image)
        
        # Plan and execute with vision
        return super().execute_mission(context_with_vision)
```

### 2. Object Detection Pipeline

Replace GPT-4V with specialized models for common tasks:
- YOLO/Detectron2 for object detection
- SAM for segmentation
- CLIP for zero-shot classification
- Use GPT-4V only for complex reasoning

### 3. Visual Memory

Store and recall previously seen objects/locations:
```python
@register_skill("vision.remember_scene")
class RememberSceneSkill(Skill):
    """Store current view with a label for later recall."""
    pass

@register_skill("vision.recall_scene")
class RecallSceneSkill(Skill):
    """Retrieve a previously stored scene description."""
    pass
```

### 4. Multi-Camera Fusion

Combine views from multiple cameras for 360° awareness.

## Implementation Checklist

- [ ] Create `shadowhound_skills` package structure
- [ ] Implement `SkillBase` abstract class
- [ ] Implement `SkillRegistry` singleton
- [ ] Create `vision.py` skills module
  - [ ] `perception.snapshot` skill
  - [ ] `vision.describe_scene` skill
  - [ ] `vision.detect_objects` skill
  - [ ] `vision.locate_object` skill
- [ ] Update `RobotInterface` with vision methods
- [ ] Wire up web interface mock images to robot interface
- [ ] Add vision skills to DIMOS skills library
- [ ] Write unit tests for each skill
- [ ] Create test images for different scenarios
- [ ] Document vision skills in agent prompts
- [ ] Test end-to-end with mock images
- [ ] Test with real robot camera

## Summary

**Recommendation:** Start with **Vision Skills (Option 1)**

This approach:
- ✅ Leverages existing mock image upload system
- ✅ Works with current agent architecture
- ✅ Testable without robot hardware
- ✅ Cost-effective (vision only when needed)
- ✅ Flexible and composable
- ✅ Clear path to production

Once vision skills are stable and we have real-world usage data, we can evaluate whether Option 3 (hybrid approach) would provide additional value.

---

**Next Steps:**
1. Review this design with the team
2. Create `shadowhound_skills` package scaffold
3. Implement basic vision skills with mock image support
4. Test with web UI image upload
5. Iterate based on testing feedback

