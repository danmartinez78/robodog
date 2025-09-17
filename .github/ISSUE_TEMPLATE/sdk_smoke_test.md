---
name: SDK — Smoke Test & Sensors
about: Validate go2_ros2_sdk nodes, URDF, and sensors
title: "SDK: smoke test (URDF, LiDAR, teleop)"
labels: ["sdk", "phase0", "testing"]
assignees: []
---

## Goals
- Build and run core nodes from `go2_ros2_sdk`
- Visualize robot model and basic sensors
- Verify control loop with teleop

## Checklist
- [ ] Build succeeds on Dev PC and Orin
- [ ] Launch base bring-up (list command used)
- [ ] RViz shows URDF and joint states
- [ ] LiDAR topic visible (expected ~7 Hz on some firmware)
- [ ] Teleop works and latency acceptable
- [ ] Capture screenshots/video for documentation

## Logs & Screens
Attach terminal logs and RViz captures.
