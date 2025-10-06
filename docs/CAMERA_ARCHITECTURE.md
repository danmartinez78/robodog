# Camera Architecture & Design Decisions

## Overview

This document explains the camera pipeline architecture for ShadowHound and the rationale behind design decisions regarding raw vs compressed image formats.

**Last Updated**: October 6, 2025  
**Status**: Current Implementation - Image Republisher Pattern

---

## Current Architecture

```
┌─────────────────┐
│  Unitree Go2    │
│  (WebRTC H.264) │
└────────┬────────┘
         │ WebRTC Video Stream
         ↓
┌─────────────────────────────────────┐
│  go2_driver_node                    │
│  - Decodes WebRTC frames            │
│  - Publishes /camera/image_raw      │
│    (sensor_msgs/Image, BGR24)       │
│  - ~720p, uncompressed              │
│  - QoS: best_effort                 │
└────────┬────────────────────────────┘
         │ Raw Images (~2-3 MB/frame)
         ↓
┌─────────────────────────────────────┐
│  image_republisher (image_transport)│
│  - Subscribes to image_raw          │
│  - Encodes to JPEG                  │
│  - Publishes /camera/compressed     │
│    (sensor_msgs/CompressedImage)    │
│  - ~100-200 KB/frame (JPEG quality) │
│  - QoS: best_effort/volatile        │
└────────┬────────────────────────────┘
         │
         ├─────────────────┬──────────────────┐
         ↓                 ↓                  ↓
    ┌─────────┐      ┌─────────┐      ┌──────────┐
    │ Web UI  │      │  VLM    │      │  Future  │
    │ (1 FPS) │      │  Vision │      │  Modules │
    └─────────┘      └─────────┘      └──────────┘
```

---

## Design Rationale

### Why Keep Raw + Compressed?

#### ✅ Advantages of Current Approach

1. **Ecosystem Compatibility**
   - RViz2 expects raw images for visualization
   - OpenCV tools often work better with raw
   - Many ROS2 perception packages expect sensor_msgs/Image

2. **Flexibility**
   - Raw for computer vision (object detection, SLAM)
   - Compressed for network efficiency (web UI, telemetry)
   - Choose format per use case

3. **Zero Maintenance Burden**
   - Uses standard ROS2 `image_transport` package
   - No custom fork of go2_ros2_sdk to maintain
   - Easy to pull upstream updates from RoboVerse

4. **Debug & Development**
   - Can inspect raw frames for quality issues
   - Tools like `rqt_image_view` work out of box
   - Better for development and troubleshooting

#### ⚠️ Tradeoffs

1. **Performance Overhead**
   - Double image processing (decode WebRTC + encode JPEG)
   - Extra CPU for JPEG compression (~30-40% of core)
   - Extra latency (~10-30ms for compression)

2. **Memory Bandwidth**
   - Raw images are 5-10x larger than compressed
   - More memory copies between nodes
   - Higher DDS/CycloneDDS network load

3. **Pipeline Complexity**
   - Extra node in the stack (image_republisher)
   - More topics to monitor and debug
   - QoS configuration must match across chain

---

## Alternative Approaches Considered

### Option 1: Compressed Only (Not Chosen)

**Modify go2_driver_node to publish only `/camera/compressed`**

```python
# In ros2_publisher.py publish_camera_data()
ros_image = self.bridge.cv2_to_compressed_imgmsg(camera.image, dst_format='jpeg')
self.publishers['camera_compressed'][robot_idx].publish(ros_image)
```

**Pros:**
- ✅ Maximum performance (single encode path)
- ✅ Lowest CPU usage
- ✅ Smallest bandwidth (~5-10x reduction)
- ✅ Simpler pipeline (one less node)

**Cons:**
- ❌ Breaks RViz2 and other raw image tools
- ❌ Requires maintaining fork of go2_ros2_sdk
- ❌ Harder to merge upstream changes
- ❌ Less flexible for future CV work

**Verdict**: Too restrictive, rejected.

---

### Option 2: Hybrid SDK Parameter (Future Consideration)

**Add configurable camera format to go2_driver_node**

```python
# In go2_driver_node.py configuration
self.declare_parameter('camera_format', 'raw')  # 'raw', 'compressed', 'both'

# In publish_camera_data()
if self.config.camera_format in ['raw', 'both']:
    ros_image = self.bridge.cv2_to_imgmsg(camera.image, encoding='bgr8')
    self.publishers['camera'][robot_idx].publish(ros_image)

if self.config.camera_format in ['compressed', 'both']:
    compressed = self.bridge.cv2_to_compressed_imgmsg(camera.image, dst_format='jpeg')
    self.publishers['camera_compressed'][robot_idx].publish(compressed)
```

**Pros:**
- ✅ Backwards compatible (default to 'raw')
- ✅ Opt-in performance mode ('compressed')
- ✅ Can publish both if needed ('both')
- ✅ Could PR upstream to RoboVerse
- ✅ Benefits entire community

**Cons:**
- ⚠️ Requires modifying go2_ros2_sdk
- ⚠️ More code to test and maintain
- ⚠️ Needs upstream buy-in

**Verdict**: Good future enhancement, could propose to RoboVerse.

---

### Option 3: Current Approach (✅ CHOSEN)

**Use standard image_transport republisher**

**Pros:**
- ✅ Zero custom code
- ✅ Standard ROS2 pattern
- ✅ Easy to optimize (quality, resolution, rate)
- ✅ No fork maintenance
- ✅ Works with all ROS2 tools

**Cons:**
- ⚠️ Some performance overhead
- ⚠️ Extra node in pipeline

**Verdict**: Best balance for MVP and maintainability.

---

## Performance Characteristics

### Measured Overhead (Estimated)

| Stage | Latency | CPU | Bandwidth |
|-------|---------|-----|-----------|
| WebRTC decode | 5-15ms | 20-30% | N/A (network) |
| Raw publish | 1-2ms | 5% | ~2-3 MB/frame |
| JPEG compress | 10-30ms | 30-40% | N/A |
| Compressed publish | 1-2ms | 5% | ~100-200 KB/frame |
| **Total** | **17-49ms** | **60-80%** | **2-3 MB/frame raw** |

**Notes:**
- Assumes 720p @ 30 FPS
- CPU percentages per core (quad-core system)
- Latency can vary with system load
- Web UI only needs 1 FPS (can throttle)

### Optimization Opportunities

**Without modifying SDK:**

1. **Lower JPEG Quality** (default 80 → 60)
   ```python
   # In robot.launch.py image_republisher parameters
   'jpeg_quality': 60  # Reduces size by ~30%, minimal visual loss
   ```

2. **Reduce Resolution** (720p → 480p)
   ```python
   # Add image_proc resize node before republisher
   # Reduces encode time by ~50%
   ```

3. **Rate Limit** (30 FPS → 10 FPS)
   ```python
   # Throttle in republisher or mission_agent
   # Web UI only needs 1-2 FPS anyway
   ```

4. **Conditional Publishing**
   ```python
   # Only publish compressed when web UI is connected
   # Check active_connections in web_interface.py
   ```

---

## QoS Configuration

### Critical Settings

Both raw and compressed topics use **best_effort/volatile** QoS to match sensor data semantics:

```python
# In robot.launch.py
parameters=[{
    'qos_overrides./camera/compressed.publisher.reliability': 'best_effort',
    'qos_overrides./camera/compressed.publisher.durability': 'volatile',
}]
```

**Why best_effort?**
- Camera is a sensor stream (late data is useless)
- Prefer fresh frames over guaranteed delivery
- Lower latency (no retransmissions)
- Matches robot's sensor QoS policy

**Why volatile?**
- No need to persist old frames
- Late joiners should get fresh data
- Reduces memory usage

### Common QoS Issues

**Problem**: Subscription fails with "incompatible QoS" error

**Solution**: Ensure subscriber matches publisher:
```python
# In mission_agent.py
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

qos = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE
)
self.camera_sub = self.create_subscription(
    CompressedImage, "/camera/compressed", self.camera_callback, qos
)
```

---

## Current Implementation

### Files Involved

1. **launch/go2_sdk/robot.launch.py** (lines 236-250)
   - Launches `image_republisher` node
   - Configures QoS overrides
   - Remaps topics

2. **mission_agent.py** (lines 95-96, 263-268)
   - Subscribes to `/camera/compressed`
   - Forwards to web_interface via `update_camera_frame()`
   - Uses best_effort QoS

3. **web_interface.py** (lines 83, 237-243, 241-247)
   - Stores `latest_camera_frame` as bytes
   - API endpoint `/api/camera/latest` returns base64 JPEG
   - Serves to web UI at 1 FPS

4. **dashboard_template.html** (lines 200-220, 490-510)
   - Displays camera feed in Matrix-themed panel
   - Polls `/api/camera/latest` every 1 second
   - Handles "no camera available" gracefully

### Testing

**Verify camera pipeline:**

```bash
# Terminal 1: Launch full stack
ros2 launch shadowhound_bringup shadowhound.launch.py

# Terminal 2: Check topics exist
ros2 topic list | grep camera
# Expected: /camera/image_raw, /camera/compressed, /camera/camera_info

# Terminal 3: Check publisher info
ros2 topic info /camera/compressed
# Expected: Publishers: 1 (image_republisher)

# Terminal 4: Monitor rate
ros2 topic hz /camera/compressed
# Expected: ~30 Hz (robot camera rate)

# Terminal 5: View in RViz
ros2 run rqt_image_view rqt_image_view /camera/image_raw

# Terminal 6: Launch mission agent
ros2 launch shadowhound_mission_agent bringup.launch.py
# Navigate to http://localhost:8080 - should see camera feed
```

---

## Future Enhancements

### Phase 1: Optimization (No SDK Changes)

- [ ] Add JPEG quality parameter to republisher (default 60)
- [ ] Add resolution downsampling (720p → 480p for web)
- [ ] Add rate throttling (30 FPS → 10 FPS)
- [ ] Conditional publishing based on subscriber count

### Phase 2: Hybrid SDK (Requires go2_ros2_sdk Fork)

- [ ] Add `camera_format` parameter to go2_driver_node
- [ ] Implement dual publishing (raw + compressed)
- [ ] Add compression quality/format options
- [ ] Performance benchmarking
- [ ] Propose PR to RoboVerse upstream

### Phase 3: Advanced (Future)

- [ ] Hardware JPEG encoding (NVENC/VAAPI)
- [ ] Multiple resolution streams
- [ ] H.264 pass-through (skip decode/re-encode)
- [ ] Dynamic quality adjustment based on bandwidth

---

## Related Documentation

- **go2_ros2_sdk**: `src/dimos-unitree/dimos/robot/unitree/external/go2_ros2_sdk/`
- **Image Transport**: http://wiki.ros.org/image_transport
- **ROS2 QoS**: https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html
- **Web UI Implementation**: `docs/WEB_UI_ENHANCEMENTS.md` (if exists)

---

## Decision Log

| Date | Decision | Rationale |
|------|----------|-----------|
| 2024-10-06 | Use image_republisher for raw→compressed | Standard ROS2 pattern, zero maintenance, good enough performance |
| 2024-10-06 | Subscribe to compressed in mission_agent | Reduces bandwidth to web UI, JPEG already optimal for display |
| 2024-10-06 | Defer SDK modification | Wait for performance issues before forking, maintain upstream compatibility |

---

## Contact & Maintenance

**Owner**: ShadowHound Team  
**Reviewers**: For questions about camera architecture  
**Upstream**: RoboVerse go2_ros2_sdk - https://github.com/RoboVerse-community/go2_ros2_sdk

**Related Issues**: (TBD when GitHub issues created)
