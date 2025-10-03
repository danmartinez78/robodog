# ShadowHound Project Context
_Last updated: 2025-10-03_

This document defines the working architecture and implementation plan for the ShadowHound robot system. The project is built on **go2_ros2_sdk** with an LLM/VLM/VLA-driven agent that executes instructions by calling modular **ROS 2 Skills**. It runs in Docker containers, supports cloud and local models, and is structured for fast iteration on both laptop and Jetson AGX Thor.

---

## 🎯 Demo Objective

**User instruction:** “Find the blue ball in the kitchen.”

The agent will:
1. Check for an existing map, localization status, and a known kitchen POI.
2. If known → navigate to kitchen directly.
3. If not → explore and label rooms until kitchen is identified.
4. In kitchen → scan using VLM or local detector for the blue ball.
5. If detected → navigate to it and report via speech + snapshot.

---

## 🧠 Architecture

```
┌──────────────────────────────────────────────────────────────┐
│ Agent (LLM / VLM / VLA)                                      │
│  • Parses instructions into skill steps                      │
│  • Uses Skills API (preferred) or limited direct ROS access  │
├──────────────────────────────────────────────────────────────┤
│ Skills Layer (ROS 2 using go2_ros2_sdk)                      │
│  • Typed, safe wrappers over ROS topics/services/actions     │
│  • Handles timeouts, clamps, safety, logging                 │
└──────────────────────────────────────────────────────────────┘
```

---

## 🔧 MVP Skills

### Navigation & Mapping
- `nav.goto(x, y, yaw)`
- `nav.rotate(yaw)`
- `nav.explore_frontiers(max_time_s)`
- `map.load(name)`
- `map.set_poi(label, pose)`
- `map.has_poi(label)`

### Perception
- `perception.snapshot() -> uri, ts`
- `perception.room_label() -> label, p`
- `perception.detect("blue ball") -> found, bbox, depth_m`

### UX
- `report.say(text)`
- `report.snapshot_and_caption(prefix)`

---

## 📜 Agent Plans (JSON format)

```json
[
  { "type": "skill", "name": "nav.goto", "args": { "x": 1.2, "y": 0.5, "yaw": 0.0 } },
  { "type": "skill", "name": "perception.snapshot" },
  { "type": "direct_ros", "topic": "/cmd_vel", "pattern": "spin", "duration": 2.0 }
]
```

---

## 🧪 Safety & Constraints

- `/cmd_vel`, `/odom`, `/imu`, and camera topics from `go2_ros2_sdk`
- All direct ROS access must be clamped, rate-limited, and logged
- All skill executions must include timeouts and telemetry
- Input size to cloud models: max 500 KB (thumbnails only)

---

## 🧱 Project Layout

```
.
├─ src/
│  ├─ go2_ros2_sdk/
│  ├─ skills_core/
│  ├─ skills_robot_iface/
│  ├─ agent_mission/
├─ launch/
│  └─ bringup.launch.py
├─ configs/
│  ├─ laptop_webrtc.yaml
│  └─ thor_ethernet.yaml
├─ docker/
│  ├─ Dockerfile.dev
│  └─ Dockerfile.thor
├─ docker-compose.yml
├─ project_context.md
├─ AGENTS.md
└─ README.md
```

---

## ⚙️ Development & Deployment

- Run via **Docker Compose** with profiles for `laptop` and `thor`
- `.env.example` defines ROS config, LLM endpoints, JPEG quality
- Agent backends:
  - `backend=cloud` (default)
  - `backend=local` with `llm_endpoint=http://localhost:8000`

---

## 🗺️ Phases

- ✅ **Phase 0 — Bring-up**: Skills and agent respond to `"rotate"` and `"say"` instructions
- ✅ **Phase 1 — Agent + Skills + Cloud**: Accept multi-step plans
- 🔜 **Phase 2 — Fully Local (Thor)**: Agent + models run on Jetson
- 🔮 **Phase 3 — Add E2E micro-skills**: Optional `policy.align_to_*` tools

---

## 🚀 Dev Commands

```bash
# Setup
vcs import src < shadowhound.repos
./scripts/ci_preflight.sh

# Launch on laptop
ros2 launch bringup.launch.py profile:=laptop_webrtc backend:=cloud

# Launch on Thor
ros2 launch bringup.launch.py profile:=thor_ethernet backend:=local llm_endpoint:=http://localhost:8000
```
