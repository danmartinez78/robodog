# DIMOS Vision Capabilities - Discovery Notes

**Date**: October 6, 2025  
**Branch**: feature/vlm-integration  
**Purpose**: Document existing DIMOS vision infrastructure to leverage for ShadowHound

---

## Summary

DIMOS has **comprehensive vision capabilities** we can leverage! We don't need to build from scratch.

### Key Finding: Two VLM Options

1. **OpenAI GPT-4 Vision** (via OpenAIAgent)
2. **Qwen VLM** (Alibaba's vision-language model)

Both are already integrated into DIMOS with streaming support!

---

## 🎯 DIMOS Vision Components

### 1. OpenAIAgent with Vision Support

**Location**: `dimos/agents/agent.py`

**Key Methods**:
```python
# Subscribe to video stream for processing
def subscribe_to_image_processing(
    self,
    frame_observable: Observable,
    query_extractor=None
) -> Disposable:
    """Process video frames with LLM vision capabilities."""
    # Handles frame encoding, API calls, response streaming
    
# Control image quality
self.image_detail = "low"  # or "high" or "auto"
```

**Features**:
- ✅ Streams video frames to vision API
- ✅ Automatic base64 encoding
- ✅ Frame saving for debugging (configurable limit)
- ✅ Quality control (low/high/auto)
- ✅ Observable/reactive patterns (RxPY)
- ✅ Thread pooling for performance
- ✅ Works with any OpenAI vision model (gpt-4-vision, gpt-4o, etc.)

**Usage Pattern**:
```python
from dimos.agents import OpenAIAgent

agent = OpenAIAgent(
    dev_name="VisionAgent",
    model_name="gpt-4o",  # Has vision built-in
    system_query="Describe what you see in this image"
)

# Subscribe to camera stream
video_stream = robot.get_ros_video_stream(fps=1)
agent.subscribe_to_image_processing(video_stream)

# Get responses
agent.get_response_observable().subscribe(
    on_next=lambda response: print(f"VLM says: {response}")
)
```

---

### 2. Qwen VLM Integration

**Location**: `dimos/models/qwen/video_query.py`

**Key Functions**:
```python
# Query a single frame (synchronous)
def query_single_frame(
    image: PIL.Image,
    query: str,
    api_key: Optional[str] = None,
    model_name: str = "qwen2.5-vl-72b-instruct"
) -> str:
    """Process a single PIL image with Qwen model."""
    
# Query from video stream (observable)
def query_single_frame_observable(
    video_observable: Observable,
    query: str,
    api_key: Optional[str] = None
) -> Observable:
    """Process a single frame from video stream."""
    
# Get bounding boxes for objects
def get_bbox_from_qwen_frame(
    frame,
    object_name: Optional[str] = None
) -> Optional[tuple]:
    """Returns (bbox, size) where bbox=[x1,y1,x2,y2]"""
```

**Features**:
- ✅ Synchronous and async (Observable) interfaces
- ✅ Bounding box detection with size estimation
- ✅ JSON structured output support
- ✅ Alibaba API integration
- ✅ Built-in error handling

**Usage Pattern**:
```python
from PIL import Image
from dimos.models.qwen.video_query import query_single_frame, get_bbox_from_qwen_frame

# Simple query
image = Image.open("robot_view.jpg")
response = query_single_frame(
    image, 
    "What objects do you see?",
    api_key=os.getenv("ALIBABA_API_KEY")
)

# Object detection with bounding box
bbox, size = get_bbox_from_qwen_frame(
    image,
    object_name="coffee cup"
)
print(f"Coffee cup at {bbox}, estimated height: {size}m")
```

---

### 3. Object Detection (Non-VLM)

**Location**: `dimos/perception/`

**Available Detectors**:

#### Detic (Open-vocabulary)
```python
from dimos.perception.detection2d.detic_2d_det import Detic2DDetector

detector = Detic2DDetector(
    vocabulary=None,  # Uses LVIS classes, or custom list
    threshold=0.5
)
bboxes, track_ids, class_ids, confidences, names = detector.process_image(frame)
```

#### YOLO
```python
from dimos.perception.detection2d.yolo_2d_det import Yolo2DDetector

detector = Yolo2DDetector(
    model_name="yolo11n.pt",
    conf_threshold=0.5
)
bboxes, track_ids, class_ids, confidences, names = detector.process_image(frame)
```

**Note**: These require CUDA for performance, but YOLO can run on CPU.

---

### 4. Object Detection Stream (Unified Pipeline)

**Location**: `dimos/perception/object_detection_stream.py`

**What it Does**:
1. Detects objects (Detic or YOLO)
2. Estimates depth (Metric3D)
3. Calculates 3D position with camera intrinsics
4. Transforms to map frame
5. Returns structured Observable stream

```python
from dimos.perception.object_detection_stream import ObjectDetectionStream

detection_stream = ObjectDetectionStream(
    camera_intrinsics=[fx, fy, cx, cy],
    device="cuda",  # or "cpu"
    min_confidence=0.5,
    class_filter=["person", "cup", "bottle"],  # Optional
    transform_to_map=robot.ros_control.transform_pose
)

# Create stream from video
video_stream = robot.get_ros_video_stream()
objects_stream = detection_stream.create_stream(video_stream)

# Subscribe to detections
objects_stream.subscribe(
    on_next=lambda obj_data: print(f"Detected: {obj_data}")
)
```

**Output Format**:
```python
{
    'name': 'person',
    'confidence': 0.87,
    'bbox': [x1, y1, x2, y2],
    'position': Vector(x, y, z),  # 3D coordinates
    'rotation': Rotation(...),     # Orientation
    'dimensions': {'width': 0.5, 'height': 1.7, 'depth': 0.3}
}
```

---

### 5. Depth Estimation

**Location**: `dimos/models/depth/metric3d.py`

```python
from dimos.models.depth.metric3d import Metric3D

depth_model = Metric3D(gt_depth_scale=1000.0)
depth_model.update_intrinsic([fx, fy, cx, cy])

# Get depth map
depth_map = depth_model.estimate_depth(frame)
```

---

### 6. Segmentation

**Location**: `dimos/perception/segmentation/`

- **SAM** (Segment Anything Model)
- **CLIPSeg** (Text-guided segmentation)

---

## 🚀 Recommendations for ShadowHound

### Option A: Use OpenAIAgent's Vision (Recommended for MVP)

**Pros**:
- ✅ Already integrated with our agent architecture
- ✅ Uses same OpenAI API key we're already using
- ✅ GPT-4o has excellent vision capabilities
- ✅ No new dependencies
- ✅ Streaming support built-in

**Cons**:
- ⚠️ OpenAI API costs (~$0.01-0.02 per image at high detail)
- ⚠️ Requires internet connection

**Implementation**:
```python
# In MissionAgentNode
def enable_vision(self):
    """Enable vision processing for the agent."""
    # Agent already has vision capability!
    # Just subscribe to camera stream
    camera_stream = self.create_camera_observable()
    self.agent.subscribe_to_image_processing(
        camera_stream,
        query_extractor=lambda frame: (self.current_vision_query, frame)
    )
```

---

### Option B: Use Qwen for Object Detection

**Pros**:
- ✅ Structured output (bounding boxes)
- ✅ Size estimation included
- ✅ Alibaba API (different cost structure)
- ✅ Simple synchronous interface

**Cons**:
- ⚠️ Requires separate ALIBABA_API_KEY
- ⚠️ Not integrated with our agent (would need wrapper)

**Use Case**: Specific object localization tasks

---

### Option C: Hybrid Approach (Best Long-term)

1. **For scene understanding**: Use OpenAIAgent with GPT-4o
2. **For object detection**: Use Qwen or YOLO
3. **For depth**: Use Metric3D if needed

---

## 📋 Implementation Plan

### Phase 1: Leverage OpenAIAgent Vision (Simplest)

**Goal**: Enable vision in existing agent with minimal code

**Tasks**:
1. Add camera frame Observable to MissionAgentNode
2. Call `agent.subscribe_to_image_processing(camera_stream)`
3. Create vision skills that set `system_query` and get responses
4. Test with web UI camera feed

**Code Changes**:
- ✅ `mission_agent.py` - Already has camera subscription!
- ✅ Just need to wire it to agent
- ⚠️ Need to create skills that trigger vision queries

---

### Phase 2: Vision Skills (Recommended Architecture)

Create skills that internally use DIMOS vision:

```python
# shadowhound_skills/vision.py

@register_skill("vision.describe_scene")
class DescribeSceneSkill(Skill):
    """Use VLM to describe current camera view."""
    
    def execute(self, **kwargs) -> SkillResult:
        # Get latest frame from mission_agent camera buffer
        frame = self.robot_interface.get_latest_camera_frame()
        
        # Use Qwen for quick synchronous response
        from dimos.models.qwen.video_query import query_single_frame
        response = query_single_frame(
            frame,
            "Describe what you see. List all objects and their approximate locations."
        )
        
        return SkillResult(
            success=True,
            data={"description": response}
        )

@register_skill("vision.locate_object")
class LocateObjectSkill(Skill):
    """Find specific object and return bounding box."""
    
    def execute(self, object_name: str, **kwargs) -> SkillResult:
        frame = self.robot_interface.get_latest_camera_frame()
        
        from dimos.models.qwen.video_query import get_bbox_from_qwen_frame
        result = get_bbox_from_qwen_frame(frame, object_name)
        
        if result is None:
            return SkillResult(
                success=False,
                error=f"Object '{object_name}' not found"
            )
        
        bbox, size = result
        return SkillResult(
            success=True,
            data={
                "object": object_name,
                "bbox": bbox,
                "estimated_size": size
            }
        )
```

---

## 🔧 Environment Setup

### OpenAI Vision (Already Configured)
```bash
export OPENAI_API_KEY="sk-..."
# Uses gpt-4o by default (has vision built-in)
```

### Qwen Vision (Optional)
```bash
export ALIBABA_API_KEY="sk-..."  # Get from Alibaba Cloud
```

### Model Costs (Approximate)

| Provider | Model | Cost per Image | Quality |
|----------|-------|---------------|---------|
| OpenAI | gpt-4o | ~$0.005 (low detail) | Excellent |
| OpenAI | gpt-4o | ~$0.015 (high detail) | Best |
| OpenAI | gpt-4-vision | ~$0.01-0.03 | Excellent |
| Alibaba | qwen2.5-vl-72b | ~$0.003 | Very Good |
| Alibaba | qwen2.5-vl-7b | ~$0.001 | Good |

---

## 🎓 Key Learnings

1. **Don't Reinvent the Wheel**: DIMOS has production-ready vision infrastructure
2. **Two API Options**: OpenAI (already using) or Alibaba Qwen (cheaper)
3. **Observable Streams**: DIMOS uses RxPY for reactive vision processing
4. **Quality Control**: `image_detail` parameter for cost/quality tradeoff
5. **Skills Pattern**: Vision should be skills, not direct agent modification

---

## 📚 Reference Files

### DIMOS Vision Code
- `dimos/agents/agent.py` - OpenAIAgent with vision
- `dimos/models/qwen/video_query.py` - Qwen VLM functions
- `dimos/perception/object_detection_stream.py` - Unified detection pipeline
- `dimos/perception/detection2d/detic_2d_det.py` - Detic detector
- `dimos/perception/detection2d/yolo_2d_det.py` - YOLO detector

### ShadowHound Design Docs
- `docs/VISION_INTEGRATION_DESIGN.md` - Original design (630 lines)
- `docs/mvp_plan_pivot.md` - MVP vision plan
- `docs/CAMERA_ARCHITECTURE.md` - Camera pipeline

---

## ✅ Next Actions

1. **Immediate**: Create vision skills using Qwen (simplest)
2. **Short-term**: Wire OpenAIAgent vision to mission execution
3. **Long-term**: Hybrid approach with object detection stream

---

**Status**: Discovery complete. Ready to implement! 🚀
