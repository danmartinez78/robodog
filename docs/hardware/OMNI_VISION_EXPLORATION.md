# 360° Vision – Comprehensive Options & Plan (v0.9)

This document consolidates our options for adding **omnidirectional visual awareness** to ShadowHound, alongside the existing **LiDAR (depth authority)** and **forward mono camera (semantics)**. The 360° layer feeds **situational context** to VLM/VLA agents and helps detect events outside the forward arc without burdening the Thor GPU.

---

## Target Architecture (recap)
- **LiDAR (forward):** real-time metric depth for planning/avoidance.
- **Mono RGB (forward):** semantic perception and VLM input for the frontal cone.
- **360° camera (top-mounted):** global awareness (rear/side), event triggers, and VLM-friendly context crops.

**Principle:** The 360° layer is *contextual*, not a primary depth sensor. Keep the LiDAR→planner loop deterministic; run 360° analytics on a sidecar path with ROI gating to preserve GPU headroom.

---

## Why DreamVu First
The **DreamVu PAL family** (PAL USB and PAL Mini) is uniquely suited to ShadowHound's needs:
- **USB-powered and lightweight** – single cable for power + data; no PoE or auxiliary regulators.
- **Small form factor** – ideal for mast or top-plate mounting without disturbing LiDAR/IMU.
- **Native ROS 2 SDK** – publishes `/pal/image_raw`, `/pal/depth`, `/pal/point_cloud` directly with timestamps for PTP sync.
- **Host-side GPU processing** – PAL USB/Mini send sensor data to the host, where the DreamVu SDK performs stereo disparity, stitching, and dewarping using CUDA/TensorRT acceleration.
- **360° RGBD coverage** – supplies both visual context and approximate range, bridging the gap between pure vision and metric LiDAR.

This makes DreamVu cameras the most balanced and integration-ready option. They deliver the broad awareness a VLM needs while keeping compute and wiring simple—perfect for continuous, low-overhead world modeling.

---

## Integration Requirements

| Area | Requirement | Notes |
|---|---|---|
| Interface | USB 3.x | Fits the wired LAN; minimizes jitter. |
| Output | Equirectangular or dual-fisheye with SDK | Already-stitched output preferred. |
| Rate | 5–15 FPS typical | Sufficient for awareness; burst on motion events. |
| Resolution | ≥ 2880×1440 live (goal 4K pano) | Enough pixels for clean ROI crops. |
| Latency | < ~80 ms end-to-end | Fine for awareness; not in the hard control loop. |
| SDK | Linux/Jetson capture path | UVC (webcam) or vendor SDK. |

Holoscan graph sketch:
```
[Omni] -> [Decode/Dewarp] -> [Light ROI Detector] --crops--> [VLM]
                                         |                       |
                                         +----tags-------------->+-> /scene_summary
```

---

## Off-the-Shelf Candidates (shortlist)

### DreamVu **PAL USB** and **PAL Mini**
- **PAL USB:** 360° × 110° FoV, 3840×1080@15 FPS, ~3.5 W. Ideal balance of cost and coverage.
- **PAL Mini:** 360° × 89° FoV, 3440×1019@10 FPS, ~2.5 W. Compact, low power, same SDK.
- **SDK:** [DreamVu PAL SDK](https://dreamvu.com/support/)
- **Processing Architecture:** Host-side; stereo disparity, dewarping, and fusion executed on Thor GPU (CUDA/TensorRT). 
- **ROS 2 Support:** ✅ [pal_camera_ros2](https://github.com/DreamVu-ROS/pal_camera_ros2)
- **Compute Load:** GPU-assisted; moderate usage (~15–30% GPU) on Jetson AGX Thor; negligible CPU.
- **Fit:** Primary candidate for integration; full RGB-D + ROS 2; best for persistent 360° awareness with manageable GPU overhead.

### Insta360 **X4**
- **Product:** https://www.insta360.com/product/insta360-x4
- **Processing Architecture:** Performs **on-device stitching**; the camera fuses the dual fisheye feeds into a single equirectangular panorama before sending the output stream over USB.
- **SDK:** [Insta360 Developer SDK](https://www.insta360.com/developer)
- **ROS 2:** ⚪ Generic UVC ingestion only.
- **Compute Load:** Minimal; host receives stitched frames and only handles decoding.
- **Fit:** Lightweight integration and zero stitching load make it an excellent baseline or secondary 360° awareness option.

---

## Calibration (Intrinsic / Extrinsic)

Accurate alignment between the omni camera, LiDAR, and robot base is essential for meaningful 3D fusion and VLM reasoning.

### Intrinsic Calibration
Performed once per camera to model lens and mirror geometry.
- **For DreamVu PAL series:** The SDK provides factory-calibrated intrinsic parameters; accessible via the API and automatically loaded by ROS 2 driver.
- **DIY / verification:** Use OpenCV's `omnidir.calibrate()` for catadioptric systems or standard fisheye calibration with a checkerboard. Validate against vendor calibration.

### Extrinsic Calibration
Defines camera position/orientation relative to LiDAR and robot base.
- **Method 1:** Use ROS 2 `tf2` and a calibration target visible to both sensors (e.g., checkerboard or AprilTag board in LiDAR view). Run `camera_lidar_calibration` or custom ICP alignment on depth ↔ point cloud.
- **Method 2:** Manual measurement for rough placement + iterative refinement using reprojection error minimization.
- **Goal:** Extrinsic transform published as `static_transform_publisher` between `/pal_optical_frame` and `/base_link`.

Accurate calibration ensures:
- Proper fusion of omni depth with LiDAR occupancy grids.
- Stable semantic mapping when VLM generates spatially-anchored scene graphs.

---

## Networking & Power
- Keep 360° camera on the wired LAN when possible (USB to Thor via hub if needed); continue using **GL-SFT1200** in AP/bridge mode.
- Power via the 140 W bank: Thor (USB-C PD); router (USB-A 5 V); USB power direct for PAL USB/Mini or X4.

---

## Validation Plan
1. **Bring-up order:** PAL USB → PAL Mini → X4.
2. **Metrics:** frame stability, latency, CPU/GPU usage, pano resolution, ROI detection success.
3. **Runtime knobs:** pano FPS 8–12; downscale 0.5–0.75; ≤ 3 ROI crops per frame.
4. **Outputs:** `/omni/scene_summary`, `/omni/roi_events`, and optional `/omni/crops/*`.

---

## Decision Heuristics
- Need **full 360° 3D** and low price → **DreamVu PAL USB**.
- Need **tiny USB-powered** → **DreamVu PAL Mini**.
- Need **lowest-friction live pano (stitched on device)** → **Insta360 X4**.

---

## DreamVu PAL USB vs PAL Mini — Detailed Comparison

| Attribute | **PAL USB** | **PAL Mini** |
|------------|-------------|---------------|
| FoV | 360° (H) × 110° (V) | 360° (H) × 89° (V) |
| Depth Range | ~10 m | ~6–8 m |
| Resolution | 3840×1080 @ 15 FPS | 3440×1019 @ 10 FPS |
| Power | 3.5 W @ 5 V | 2.5 W @ 5 V |
| Size | ~93 × 64 × 47 mm | ~64 × 39 × 33 mm |
| Weight | ~220 g | ~120 g |
| Interface | USB 3.0 | USB 3.0 |
| Processing | Host GPU (CUDA/TensorRT) | Host GPU (CUDA/TensorRT) |
| ROS 2 SDK | ✅ | ✅ |
| Compute Load | Moderate GPU, low CPU | Moderate GPU, low CPU |
| Cost | Slightly lower | Slightly higher per volume |
| Mounting | Requires small bracket | Direct top-plate mount possible |
| Depth Fidelity | Higher | Slightly noisier beyond 5 m |

### Recommendation Summary
- **PAL Mini** excels where **size, weight, and simplicity** are top priorities—ideal for mounting on the Go2's top plate or mast. It maintains 360° coverage, has low power draw, and manageable GPU usage on Thor.
- **PAL USB** offers marginally greater vertical FoV and depth range, but at almost double the mass and bulk.
- **Insta360 X4** offers **true on-device stitching**, freeing Thor from image fusion tasks—ideal if simplicity and bandwidth efficiency outweigh depth needs.
- For **ShadowHound's indoor, VLM/VLA-driven architecture**, the PAL Mini remains the **best strategic choice** for near-term integration, while the X4 remains a compelling future swap if we decide to prioritize zero-host-processing simplicity over RGBD context.

---

## Open Questions
- Measure PAL USB and PAL Mini latency and GPU utilization under the CUDA SDK.
- Evaluate PAL USB vs LiDAR fusion depth quality.
- Benchmark X4's latency and compression artifacts under live UVC to ensure suitability for Holoscan.

---

*Document prepared for ShadowHound Project — 360° Vision Integration Plan (v0.9)*
