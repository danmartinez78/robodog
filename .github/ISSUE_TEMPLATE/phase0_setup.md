---
name: Phase 0 — System Bring-Up
about: Track tasks for initial bring-up on GO2 + Orin + Thor/PC
title: "Phase 0: Bring-up checklist"
labels: ["phase0", 'setup']
assignees: []
---

## Goals
- Build & run `go2_ros2_sdk` on Dev PC and Orin
- Verify wired Ethernet + Wi‑Fi dual interface
- See topics in RViz and teleop the robot

## Checklist
- [ ] Create `shadowhound/dev` branch
- [ ] Add `docs/project_context.md`
- [ ] Install ROS 2 (Iron/Humble) on Dev PC
- [ ] Flash Orin Nano (JetPack) + ROS 2 matching distro
- [ ] Clone fork with `--recurse-submodules`
- [ ] Build workspace without errors (`colcon build`)
- [ ] Run minimal nodes; verify `/tf`, joint states, LiDAR
- [ ] Teleop test (joystick/keyboard)
- [ ] Record a short demo clip for social media

## Notes
Add logs, screenshots, and console outputs here.
