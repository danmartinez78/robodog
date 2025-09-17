# ShadowHound Project Context
_Last updated: 2025-09-16_

## 🧠 Project Vision
ShadowHound is a general-purpose, interactive robotic platform built on a Unitree GO2 Pro quadruped with the following goals:

- Operate in indoor environments (home, lab, office)
- Interact naturally with humans via voice and language
- Perceive, reason, and act using transformer-based AI models
- Learn and adapt over time
- Serve as a content-ready showcase on social media

---

## 🤖 Hardware Stack

| Component          | Role                                      |
|--------------------|-------------------------------------------|
| **GO2 Pro (jailbroken)** | Mobile base + low-level sensors and actuators |
| **Jetson Orin Nano 8GB** | Mounted on GO2; runs local autonomy stack (SLAM, perception, control) |
| **Jetson AGX Thor**      | High-power compute for LLM/VLM/VLA models |
| **GPU Desktop PC**       | Dev box and/or secondary model host (LLMs, training, visualization) |

---

## 📡 Networking Design

### Primary Topology (Recommended)
- GO2 <--> Orin Nano via Ethernet (static IPs, e.g., 192.168.50.2 / .3)
- Orin connects to Wi‑Fi for SSH, updates, and remote debugging
- Dev laptop/Thor connects to the same Wi‑Fi network
- Optional: Use onboard travel router and switch for fully mobile LAN

### ROS 2 Graph Discovery
- ROS_DOMAIN_ID: 7 (consistent across GO2, Orin, and laptop)
- RMW: CycloneDDS or FastDDS (with optional discovery server)
- Topics visible over Wi‑Fi unless restricted in DDS config

---

## 🧠 Reasoning Stack

### Task Split

| Component     | Role |
|---------------|------|
| **Orin Nano** | Local autonomy, SLAM, planner execution, request builder |
| **Thor/PC**   | Hosts LLM/VLM/VLA models for planning and high-level reasoning |
| **GO2**       | Provides physical movement and raw sensor input |

### Request Flow

1. Orin builds structured request (scene graph + thumbnails + instruction)
2. Sends to Thor/PC via gRPC
3. Thor/PC responds with a HighLevelPlan (waypoints, goals, text)
4. Orin executes via local planners and control nodes

### Model Serving

- Offloaded via gRPC or HTTP (e.g., Triton, vLLM, TensorRT-LLM)
- LLMs (e.g., Qwen2.5, Llama 3, Mixtral)
- VLMs (e.g., LLaVA, Qwen-VL, OpenVLA)
- Keep ROS 2 interaction lightweight (metadata only)

---

## 🗂 Reference Repositories

- https://github.com/abizovnuralem/go2_ros2_sdk — Base SDK for GO2 ROS 2 integration
- https://github.com/Jen-Hung-Ho/ros2_jetbot_tools — Jetson-based ROS 2 utilities for headless setups and config
- https://github.com/Jen-Hung-Ho/ros2_jetbot_voice — Voice interaction system for Jetson robots (ASR + TTS)

---

## 🔜 Project Phases

### ✅ Phase 0: System Bring-Up (current)
- Fork and initialize go2_ros2_sdk
- Set up ROS 2 on Orin Nano (headless)
- Establish hardwired + Wi‑Fi networking
- Smoke test URDF, teleop, and sensor feeds

### 🗣️ Phase 1: Interactive Skeleton
- Add request_builder node
- Connect to dummy gRPC server (reasoner stub)
- Visualize graph, commands, and responses

### 🧠 Phase 2: Reasoning Loop
- LLM/VLM integrated on Thor/PC
- Chain of reasoning → multi-step plan execution

### 👀 Phase 3: Grounded Perception + Memory
- Scene graph construction, object memory, grounding language to map

### 🤖 Phase 4: Online Learning & Adaptation
- Vector memory, few-shot learning, memory recall, and long-term model updates

---

## 📌 Notes

- All ROS 2 devices must match on distro (Iron or Humble) and RMW
- Use gRPC over HTTP/2 for reasoning interfaces (avoid pushing large blobs over ROS topics)
- Avoid multicast dropouts by pinning DDS to wired interface or using discovery servers
- Travel router recommended for field reliability and local DNS/routing
