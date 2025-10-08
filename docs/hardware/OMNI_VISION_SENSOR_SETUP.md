# Omni Vision + Context Layer (360° / Parabolic Camera)

## Purpose & Role in Sensor Stack

This document describes the proposed **omnidirectional vision layer** for the ShadowHound system. The configuration complements the existing sensors as follows:

- **LiDAR (forward)** – Metric depth for navigation and obstacle avoidance.
- **Mono Camera (forward)** – Semantic understanding and VLM (Vision-Language Model) input for object recognition.
- **Omnidirectional / Parabolic Camera (top-mounted)** – 360° situational awareness for contextual reasoning and rear/side monitoring.

The omni camera does **not replace** the RealSense or LiDAR depth system—it expands environmental perception to support large-language/vision models (LLMs/VLMs/VLAs) and multi-agent awareness.

---

## Functional Requirements

| Spec | Target | Notes |
|------|---------|-------|
| **Coverage** | 360° horizontal × ≥160° vertical | Full situational awareness, especially for confined indoor environments |
| **Interface** | Ethernet (PoE) or USB 3.0 | Seamless integration with existing wired router network |
| **Framerate** | 5–15 FPS | Sufficient for awareness and VLM context capture |
| **Resolution** | ≥ 4K (4096×2048) equirectangular equivalent | Needed for accurate region-of-interest cropping |
| **Power** | ≤ 15 W typical | Compatible with PoE+ injector and power bank setup |
| **Output** | Real-time panoramic / equirectangular | Reduces need for software stitching on Thor |
| **SDK Support** | Linux / ROS2 / GStreamer / Holoscan | Ease of integration with Jetson AGX Thor |

---

## Integration Strategy

1. **Mounting** – Install the camera at the robot's top center using a low-profile bracket to reduce occlusion by the body.
2. **Networking** – Connect via Gigabit Ethernet to the GL-SFT1200 router (PoE injector inline if required).
3. **Synchronization** – Use Precision Time Protocol (PTP) to align timestamps with LiDAR and mono camera.
4. **Processing Pipeline (Holoscan)**
   - Capture equirectangular frame.
   - Downsample and run motion/ROI detector.
   - Extract ROI crops for semantic/VLM inference.
   - Publish context tags (`/scene_summary`, `/roi_events`).
5. **Power Distribution** – Share 140 W power bank: Thor (USB-C PD), Router (USB-A), Omni cam via PoE injector.

---

## Candidate Off-the-Shelf Cameras

Below is a curated list of **parabolic and 360° cameras** suitable for indoor robotics with Jetson-class hardware.

| Model | Type | Interface | Power | Resolution | SDK / Notes |
|--------|------|------------|--------|-------------|--------------|
| **E-Con Systems e-CAM82_USB (Fisheye)** | Wide-FOV USB | USB 3.1 | 2.5 W | 8 MP | UVC compliant, GStreamer support, <https://www.e-consystems.com/usb-cameras/8mp-usb-camera.asp> |
| **RICOH Theta X** | Dual-fisheye 360° | USB-C / Wi-Fi | 7 W | 11K | Live HDMI/USB output, real-time stitching, <https://www.ricoh-imaging.co.jp/english/products/theta-x/> |
| **Insta360 Link / X3 (Industrial SDK)** | Dual-fisheye 360° | USB 3.0 | 5 W | 5.7K | SDK available for Linux, <https://www.insta360.com/product/insta360-x3> |
| **Vivotek FE9380-HV** | Industrial PoE fisheye | PoE (802.3af) | 10 W | 5 MP | RTSP/ONVIF streaming, IP66 rated, <https://www.vivotek.com/fe9380-hv> |
| **Axis M3058-PLVE** | Industrial 360° PoE | PoE+ | 12.9 W | 12 MP | Great SDK, built-in dewarping, <https://www.axis.com/products/axis-m3058-plve> |
| **See3CAM_CU135 (12.3MP 4K HDR)** | USB 3.1 | 2.5 W | 4K | Compatible with Jetson Linux drivers, <https://www.e-consystems.com/usb-cameras/12mp-usb-camera.asp> |
| **FisheyeDome-360 (OEM Catadioptric)** | Mirror-based parabolic | USB 3.0 / HDMI | 8 W | 4K | Compact, ideal for omnidirectional indoor rigs, <https://www.fisheyecam.com/products/fisheyedome-360> |

> **Recommendation:** For initial integration and VLM testing, start with the **RICOH Theta X** (best SDK + USB-C) or **Axis M3058-PLVE** (PoE, industrial stability). Both support Linux RTSP streaming and are easily integrated via GStreamer or Holoscan camera operators.

---

## Holoscan Graph Overview

```text
[LiDAR] → [Occupancy Map] → [Planner]
   ↑
[Mono Cam] → [Detector]

[Omni Cam] → [Decode → Dewarp] → [ROI Detector]
                              └─► [VLM / Scene Summarizer] → /scene_summary
```

- Omni cam processes in parallel and contributes **contextual awareness**, not real-time collision data.
- All sensors synchronized via PTP.

---

## Deployment Notes
- PoE injector (UCTRONICS U6116) recommended for industrial cams.
- 4K@10FPS 360° video ≈ 250–400 Mbps, fits within wired GigE.
- Optional: add lightweight motion detector node to trigger selective VLM crops.

---

## Next Steps
1. Acquire and bench-test **Theta X** or **Axis M3058-PLVE**.
2. Validate Holoscan integration via RTSP or USB pipeline.
3. Develop ROI detection / selective VLM inference module.
4. Conduct power and thermal profiling under continuous capture.

---

*Document prepared for the ShadowHound Project — Omni Vision Integration Layer (v0.1)*
